#include "Placement.h"
#include "BoundsUtil.h"
#include "InputBlocker.h"
#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"
#include "VRInputHandler.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>

namespace
{
	struct Quaternion
	{
		float w, x, y, z;
	};

	Quaternion MakeAxisAngle(const RE::NiPoint3& axisIn, float angle)
	{
		RE::NiPoint3 axis = axisIn;
		float len2 = axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
		if (len2 < 1e-8f) {
			return { 1.0f, 0.0f, 0.0f, 0.0f };
		}

		float invLen = 1.0f / std::sqrt(len2);
		axis.x *= invLen;
		axis.y *= invLen;
		axis.z *= invLen;

		float half = angle * 0.5f;
		float s = std::sin(half);
		float c = std::cos(half);

		return { c, axis.x * s, axis.y * s, axis.z * s };
	}

	Quaternion Mul(const Quaternion& a, const Quaternion& b)
	{
		return {
			a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
			a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
			a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
			a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
		};
	}

	RE::NiPoint3 Rotate(const Quaternion& q, const RE::NiPoint3& v)
	{
		// q * v * q^{-1} optimized
		RE::NiPoint3 qv{ q.x, q.y, q.z };

		// t = 2 * cross(qv, v)
		RE::NiPoint3 t{
			2.0f * (qv.y * v.z - qv.z * v.y),
			2.0f * (qv.z * v.x - qv.x * v.z),
			2.0f * (qv.x * v.y - qv.y * v.x)
		};

		// v' = v + q.w * t + cross(qv, t)
		RE::NiPoint3 crossQT{
			qv.y * t.z - qv.z * t.y,
			qv.z * t.x - qv.x * t.z,
			qv.x * t.y - qv.y * t.x
		};

		return RE::NiPoint3{
			v.x + q.w * t.x + crossQT.x,
			v.y + q.w * t.y + crossQT.y,
			v.z + q.w * t.z + crossQT.z
		};
	}

	RE::NiPoint3 Normalize(const RE::NiPoint3& v)
	{
		float len2 = v.x * v.x + v.y * v.y + v.z * v.z;
		if (len2 < 1e-8f)
			return RE::NiPoint3{};
		float invLen = 1.0f / std::sqrt(len2);
		return RE::NiPoint3{ v.x * invLen, v.y * invLen, v.z * invLen };
	}

	RE::NiPoint3 Cross(const RE::NiPoint3& a, const RE::NiPoint3& b)
	{
		return RE::NiPoint3{
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		};
	}

	// simple linear interpolation
	RE::NiPoint3 Lerp(const RE::NiPoint3& a, const RE::NiPoint3& b, float t)
	{
		return RE::NiPoint3{
			a.x + (b.x - a.x) * t,
			a.y + (b.y - a.y) * t,
			a.z + (b.z - a.z) * t
		};
	}

	// Mod event helper
	void SendModEvent(std::string_view eventName,
		std::string_view strArg = "",
		float numArg = 0.0f,
		RE::TESForm* sender = nullptr)
	{
		SKSE::ModCallbackEvent e{ eventName.data(), strArg.data(), numArg, sender };
		SKSE::GetModCallbackEventSource()->SendEvent(&e);
	}

	// Pivot finder:
	//  - Only returns NiNode children of the root whose subtree has geometry
	//  - Never returns the root or NiGeometry directly.
	RE::NiAVObject* FindPivotNode(RE::TESObjectREFR* ref)
	{
		if (!ref)
			return nullptr;

		auto* root3D = ref->Get3D();
		if (!root3D)
			return nullptr;

		auto* rootNode = root3D->AsNode();
		if (!rootNode)
			return nullptr;

		std::function<bool(RE::NiAVObject*)> hasGeometryRecursive;
		hasGeometryRecursive = [&](RE::NiAVObject* obj) -> bool {
			if (!obj)
				return false;

			if (obj->AsGeometry())
				return true;

			if (auto* node = obj->AsNode()) {
				for (auto& child : node->children) {
					if (child && hasGeometryRecursive(child.get()))
						return true;
				}
			}
			return false;
		};

		for (auto& child : rootNode->children) {
			if (!child)
				continue;

			auto* obj = child.get();
			auto* node = obj->AsNode();
			if (!node)
				continue;  // pure geometry; not a pivot candidate

			auto* name = obj->name.c_str();
			if (name &&
				std::string_view(name).find("Collision") != std::string_view::npos)
				continue;

			if (hasGeometryRecursive(node)) {
				return node;
			}
		}

		return nullptr;
	}
}

namespace Placement
{
	static PlacementState g_state;
	constexpr float DISTANCE_STEP = 2.0f;

	class PlacementUpdateHandler :
		public RE::BSTEventSink<RE::MenuOpenCloseEvent>
	{
	public:
		static PlacementUpdateHandler* GetSingleton()
		{
			static PlacementUpdateHandler instance;
			return std::addressof(instance);
		}

		RE::BSEventNotifyControl ProcessEvent(
			const RE::MenuOpenCloseEvent*,
			RE::BSTEventSource<RE::MenuOpenCloseEvent>*) override
		{
			if (!g_state.active || !g_state.placedRef)
				return RE::BSEventNotifyControl::kContinue;

			auto input = VRInputHandler::GetSingleton();
			if (!input)
				return RE::BSEventNotifyControl::kContinue;

			constexpr float angleStep = 2.5f * std::numbers::pi_v<float> / 180.0f;

			bool rightGrip = input->IsRightGripPressed();

			// Check for rotation reset (both joystick buttons pressed)
			if (input->IsLeftJoystickButtonPressed() && input->IsRightJoystickButtonPressed()) {
				// Reset rotation to face player (yaw=0) with no pitch/roll
				g_state.previewYaw = 0.0f;
				g_state.previewPitch = 0.0f;
				g_state.previewRoll = 0.0f;
				// Distance is NOT reset
				RE::DebugNotification("Rotation Reset");
			}

			if (rightGrip) {
				// Right grip held: joystick Y controls pitch, distance disabled
				float joyY = input->GetRightJoystickY();
				if (std::abs(joyY) > 0.1f) {
					// Pitch control via joystick (may need sensitivity adjustment)
					g_state.previewPitch += joyY * angleStep * 1.5f;  // 2x sensitivity for joystick
				}
				// Triggers do nothing when grip held
			} else {
				// No grip held: triggers control yaw, joystick controls distance
				if (input->IsLeftTriggerPressed()) {
					g_state.previewYaw -= angleStep;
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewYaw += angleStep;
				}

				// Distance control via right joystick Y (only when grip NOT held)
				float joyY = input->GetRightJoystickY();
				if (std::abs(joyY) > 0.1f) {
					g_state.previewDistance += joyY * DISTANCE_STEP;
					g_state.previewDistance =
						std::clamp(g_state.previewDistance, 50.0f, 3000.0f);
				}
			}

			LivePlace(g_state.placedRef,
				g_state.previewYaw,
				g_state.previewPitch,
				g_state.previewRoll,
				g_state.previewDistance);

			if (input->IsAButtonPressed()) {
				auto* ref = g_state.placedRef;
				if (ref) {
					ref->SetMotionType(RE::hkpMotion::MotionType::kDynamic, true);
					RE::DebugNotification("Item placement confirmed");
					OnPlacementConfirmed(ref);
				}

				g_state.active = false;
				g_state.placedRef = nullptr;
			}

			input->Reset();
			return RE::BSEventNotifyControl::kContinue;
		}

		void LivePlace(RE::TESObjectREFR* a_refr,
			float yawOffset,
			float pitchOffset,
			float rollOffset,
			float distance)
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (!player || !player->RightWandNode || !a_refr)
				return;

			// --- Position: wand + distance ---

			auto* wandNode = player->RightWandNode.get();
			const auto& wandTransform = wandNode->world;
			const auto& wandPos = wandTransform.translate;

			RE::NiPoint3 wandLocalForward{ 0.0f, 0.0f, -1.0f };
			RE::NiPoint3 wandWorldForward = wandTransform.rotate * wandLocalForward;

			RE::NiPoint3 targetCenter = wandPos + (wandWorldForward * distance);
			targetCenter.x += g_state.previewXoffset;
			targetCenter.z += g_state.previewZoffset;

			g_state.currentPreviewPos =
				Lerp(g_state.currentPreviewPos, targetCenter, g_state.positionSmoothAlpha);

			RE::NiPoint3 appliedCenter = g_state.currentPreviewPos;

			// Smooth rotation angles
			float alpha = g_state.rotationSmoothAlpha;
			g_state.currentYaw += (yawOffset - g_state.currentYaw) * alpha;
			g_state.currentPitch += (pitchOffset - g_state.currentPitch) * alpha;
			g_state.currentRoll += (rollOffset - g_state.currentRoll) * alpha;

			// Use smoothed values for actual placement
			yawOffset = g_state.currentYaw;
			pitchOffset = g_state.currentPitch;
			rollOffset = g_state.currentRoll;

			// --- Dynamic base frame: lock to player's yaw ---

			float playerYaw = player->data.angle.z;  // radians
			RE::NiPoint3 baseForward{
				std::sin(playerYaw),
				std::cos(playerYaw),
				0.0f
			};
			baseForward = Normalize(baseForward);
			RE::NiPoint3 baseUp{ 0.0f, 0.0f, 1.0f };
			RE::NiPoint3 baseRight = Normalize(Cross(baseForward, baseUp));

			if (g_state.useRootAngle) {
				//
				// Root-only geometry: drive orientation via SetAngle on the ref.
				//
				RE::NiPoint3 originPos = appliedCenter;

				float yawWorld = playerYaw + yawOffset;
				float pitchWorld = pitchOffset;
				float rollWorld = rollOffset;

				SKSE::GetTaskInterface()->AddTask(
					[a_refr, originPos, yawWorld, pitchWorld, rollWorld] {
						if (!a_refr)
							return;

						a_refr->SetPosition(originPos);
						// Skyrim: x = pitch, y = roll, z = yaw
						a_refr->SetAngle(RE::NiPoint3{ pitchWorld, rollWorld, yawWorld });
						a_refr->Update3DPosition(true);
					});

			} else {
				//
				// Child-pivot: keep root rotation unchanged and rotate pivot node only.
				//
				// 1) Yaw around baseUp
				Quaternion qYaw = MakeAxisAngle(baseUp, yawOffset);

				// Axes after yaw
				RE::NiPoint3 yawForward = Normalize(Rotate(qYaw, baseForward));
				RE::NiPoint3 yawRight = Normalize(Rotate(qYaw, baseRight));

				// 2) Pitch around right AFTER yaw
				Quaternion qPitch = MakeAxisAngle(yawRight, pitchOffset);
				Quaternion qYawPitch = Mul(qPitch, qYaw);

				// Axes after yaw + pitch
				RE::NiPoint3 yawPitchForward = Normalize(Rotate(qYawPitch, baseForward));

				// 3) Roll around forward AFTER yaw + pitch
				Quaternion qRoll = MakeAxisAngle(yawPitchForward, rollOffset);
				Quaternion qOrient = Mul(qRoll, qYawPitch);

				// Final basis (only used for building a matrix; axis values not needed here)
				RE::NiPoint3 forwardTilted = Normalize(Rotate(qOrient, baseForward));
				RE::NiPoint3 upTilted = Normalize(Rotate(qOrient, baseUp));
				RE::NiPoint3 rightTilted = Normalize(Cross(forwardTilted, upTilted));

				(void)forwardTilted;
				(void)upTilted;
				(void)rightTilted;

				RE::NiPoint3 originPos = appliedCenter;
				if (g_state.hasPivot) {
					originPos.x -= g_state.pivotOffsetWorld.x;
					originPos.y -= g_state.pivotOffsetWorld.y;
					originPos.z -= g_state.pivotOffsetWorld.z;
				}

				RE::NiPointer<RE::NiAVObject> pivotNode = g_state.pivotNode;

				float qW = qOrient.w;
				float qX = qOrient.x;
				float qY = qOrient.y;
				float qZ = qOrient.z;

				SKSE::GetTaskInterface()->AddTask(
					[a_refr, originPos, pivotNode, qW, qX, qY, qZ] {
						if (!a_refr)
							return;

						// Move the ref so pivot ends up at appliedCenter.
						// Don't touch root rotation; game owns that.
						a_refr->SetPosition(originPos);

						if (pivotNode) {
							float w = qW;
							float x = qX;
							float y = qY;
							float z = qZ;

							float xx = x * x;
							float yy = y * y;
							float zz = z * z;
							float xy = x * y;
							float xz = x * z;
							float yz = y * z;
							float wx = w * x;
							float wy = w * y;
							float wz = w * z;

							RE::NiMatrix3 rel{};
							rel.entry[0][0] = 1.0f - 2.0f * (yy + zz);
							rel.entry[0][1] = 2.0f * (xy - wz);
							rel.entry[0][2] = 2.0f * (xz + wy);

							rel.entry[1][0] = 2.0f * (xy + wz);
							rel.entry[1][1] = 1.0f - 2.0f * (xx + zz);
							rel.entry[1][2] = 2.0f * (yz - wx);

							rel.entry[2][0] = 2.0f * (xz - wy);
							rel.entry[2][1] = 2.0f * (yz + wx);
							rel.entry[2][2] = 1.0f - 2.0f * (xx + yy);

							RE::NiMatrix3 final = rel;
							if (g_state.hasPivotOriginalRotate) {
								RE::NiMatrix3 out{};
								for (int r = 0; r < 3; ++r) {
									for (int c = 0; c < 3; ++c) {
										out.entry[r][c] =
											rel.entry[r][0] * g_state.pivotOriginalRotate.entry[0][c] +
											rel.entry[r][1] * g_state.pivotOriginalRotate.entry[1][c] +
											rel.entry[r][2] * g_state.pivotOriginalRotate.entry[2][c];
									}
								}
								final = out;
							}

							pivotNode->local.rotate = final;
						}

						a_refr->Update3DPosition(true);
					});
			}
		}
	};

	void StartLivePlace(RE::TESObjectREFR* placedRef,
		float faceRotationDeg,
		float yMult,
		float zOffset,
		float xOffset)
	{
		if (!placedRef)
			return;

		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player || !player->RightWandNode)
			return;

		auto bounds = BoundsUtil::GetApproxBounds(placedRef);

		// Decide root-only vs child-pivot
		RE::NiAVObject* root3D = placedRef->Get3D();
		RE::NiAVObject* pivot = FindPivotNode(placedRef);

		RE::NiPoint3 pivotWorld{};
		RE::NiMatrix3 identity{};
		identity.entry[0][0] = 1.0f;
		identity.entry[1][1] = 1.0f;
		identity.entry[2][2] = 1.0f;

		g_state.pivotNode.reset();
		g_state.hasPivot = false;
		g_state.pivotOriginalRotate = identity;
		g_state.hasPivotOriginalRotate = false;
		g_state.pivotOffsetWorld = RE::NiPoint3{};
		g_state.useRootAngle = false;

		if (pivot) {
			// child NiNode pivot under root
			pivotWorld = pivot->world.translate;
			g_state.pivotNode.reset(pivot);
			g_state.hasPivot = true;
			g_state.pivotOriginalRotate = pivot->local.rotate;
			g_state.hasPivotOriginalRotate = true;

			RE::NiPoint3 originWorld = placedRef->GetPosition();
			g_state.pivotOffsetWorld = RE::NiPoint3{
				pivotWorld.x - originWorld.x,
				pivotWorld.y - originWorld.y,
				pivotWorld.z - originWorld.z
			};
			g_state.useRootAngle = false;
		} else {
			// no NiNode pivot: treat as root-only geometry
			pivotWorld = bounds.center;
			g_state.useRootAngle = true;
		}

		// Initial base yaw from pivot to player (for initial facing)
		RE::NiPoint3 playerPos = player->GetPosition();
		RE::NiPoint3 toPlayer{
			playerPos.x - pivotWorld.x,
			playerPos.y - pivotWorld.y,
			0.0f
		};

		float len2 = toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y;
		if (len2 < 1e-8f) {
			toPlayer = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
		} else {
			float invLen = 1.0f / std::sqrt(len2);
			toPlayer.x *= invLen;
			toPlayer.y *= invLen;
		}

		float baseYaw = std::atan2(toPlayer.x, toPlayer.y);

		g_state.baseYaw = baseYaw;
		g_state.baseForward = RE::NiPoint3{ std::sin(baseYaw), std::cos(baseYaw), 0.0f };
		g_state.baseUp = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };

		// Initial preview center
		auto* wandNode = player->RightWandNode.get();
		auto& wandTransform = wandNode->world;
		auto& wandPos = wandTransform.translate;
		RE::NiPoint3 wandLocalForward{ 0.0f, 0.0f, -1.0f };
		RE::NiPoint3 wandWorldForward = wandTransform.rotate * wandLocalForward;

		float defaultDist =
			bounds.radius > 0.0f ? bounds.radius * 2.0f : 200.0f;
		float startDist = (yMult > 0.0f) ? yMult : defaultDist;

		RE::NiPoint3 initialCenter = wandPos + (wandWorldForward * startDist);
		initialCenter.x += xOffset;
		initialCenter.z += zOffset + 150.0f;

		placedRef->SetMotionType(RE::hkpMotion::MotionType::kKeyframed, false);

		g_state.placedRef = placedRef;
		g_state.active = true;
		g_state.previewYaw = faceRotationDeg * std::numbers::pi_v<float> / 180.0f;
		g_state.previewPitch = 0.0f;
		g_state.previewRoll = 0.0f;
		g_state.previewXoffset = xOffset;
		g_state.previewZoffset = zOffset + 150.0f;
		g_state.previewDistance = startDist;

		g_state.currentPreviewPos = initialCenter;

		// Initialize smoothed rotation state
		g_state.currentYaw = g_state.previewYaw;
		g_state.currentPitch = g_state.previewPitch;
		g_state.currentRoll = g_state.previewRoll;

		if (auto* ui = RE::UI::GetSingleton()) {
			ui->AddEventSink<RE::MenuOpenCloseEvent>(
				PlacementUpdateHandler::GetSingleton());
		}

		VRInputHandler::Register();
		InputBlocker::EnterEditMode();
		RE::DebugNotification("Live Placement Started");
	}

	void OnPlacementConfirmed(RE::TESObjectREFR* a_refr)
	{
		SKSE::GetTaskInterface()->AddTask(
			[a_refr] { SendModEvent("SBOnPlacementConfirmed", "Done", 0.0f, a_refr); });

		VRInputHandler::UnRegister();

		if (auto* ui = RE::UI::GetSingleton()) {
			ui->RemoveEventSink<RE::MenuOpenCloseEvent>(
				PlacementUpdateHandler::GetSingleton());
		}

		InputBlocker::ExitEditMode();
		logger::info("Placement Done");
	}

	bool ApplyPreviewTransforms(RE::TESObjectREFR* previewRef,
		RE::TESObjectREFR* finalRef)
	{
		if (!previewRef || !finalRef)
			return false;

		previewRef->Update3DPosition(true);
		finalRef->Update3DPosition(true);

		auto* preview3D = previewRef->Get3D();
		auto* final3D = finalRef->Get3D();
		if (!preview3D || !final3D)
			return false;

		auto* previewPivot = FindPivotNode(previewRef);
		auto* finalPivot = FindPivotNode(finalRef);

		if (previewPivot && finalPivot) {
			// child-pivot case: copy pivot node's local rotation
			finalPivot->local.rotate = previewPivot->local.rotate;
			finalRef->Update3DPosition(true);
			return true;
		}

		// root-only / no pivot: copy root transform
		finalRef->SetPosition(previewRef->GetPosition());
		finalRef->SetAngle(previewRef->GetAngle());
		finalRef->Update3DPosition(true);

		return true;
	}
}
