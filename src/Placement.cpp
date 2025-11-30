#include "Placement.h"
#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"
#include "VRInputHandler.h"
#include "InputBlocker.h"
#include "BoundsUtil.h"
#include <cmath>


namespace
{

	RE::TESObjectREFR* SpawnDebugMarkerAt(const RE::NiPoint3& pos)
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player)
			return nullptr;

		// Tiny candlehorn works well as a pivot marker (FormID 0x0001f24a)
		constexpr RE::FormID candleFormID = 0x0001f24a;
		auto* baseObj = RE::TESForm::LookupByID<RE::TESBoundObject>(candleFormID);
		if (!baseObj)
			return nullptr;

		// Place at runtime
		RE::NiPointer<RE::TESObjectREFR> marker = player->PlaceObjectAtMe(
			baseObj,
			false  // forcePersist
		);

		if (!marker)
			return nullptr;

		marker->SetScale(0.1f);  // very small
		marker->SetPosition(pos);

		return marker.get();
	}


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

	float Dot(const RE::NiPoint3& a, const RE::NiPoint3& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
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

	// shortest signed angle between a and b around axis n
	float SignedAngleAroundAxis(const RE::NiPoint3& aIn,
		const RE::NiPoint3& bIn,
		const RE::NiPoint3& axisIn)
	{
		RE::NiPoint3 axis = Normalize(axisIn);
		RE::NiPoint3 a = aIn - axis * Dot(axis, aIn);
		RE::NiPoint3 b = bIn - axis * Dot(axis, bIn);
		a = Normalize(a);
		b = Normalize(b);

		float dot = std::clamp(Dot(a, b), -1.0f, 1.0f);
		float angle = std::acos(dot);
		RE::NiPoint3 cross = Cross(a, b);
		float sign = (Dot(cross, axis) >= 0.0f) ? 1.0f : -1.0f;
		return angle * sign;
	}
}


namespace Placement
{
    static PlacementState g_state;
	constexpr float DISTANCE_STEP = 2.0f;  // units per frame per full axis

	static void SendModEvent(std::string_view eventName, std::string_view strArg = "", float numArg = 0.0f, RE::TESForm* sender = nullptr)
	{
		// BSFixedString can be built from const char*
		SKSE::ModCallbackEvent e{ eventName.data(), strArg.data(), numArg, sender };
		SKSE::GetModCallbackEventSource()->SendEvent(&e);
	}

	// simple linear interpolation
	static RE::NiPoint3 Lerp(const RE::NiPoint3& a, const RE::NiPoint3& b, float t)
	{
		return RE::NiPoint3{ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
	}

	// shortest angular difference in radians
	static float ShortestAngleDiff(float from, float to)
	{
		float diff = fmodf(to - from, 2.0f * std::numbers::pi_v<float>);
		if (diff < -std::numbers::pi_v<float>)
			diff += 2.0f * std::numbers::pi_v<float>;
		else if (diff > std::numbers::pi_v<float>)
			diff -= 2.0f * std::numbers::pi_v<float>;
		return diff;
	}

class PlacementUpdateHandler : public RE::BSTEventSink<RE::MenuOpenCloseEvent>
	{
	public:
		static PlacementUpdateHandler* GetSingleton()
		{
			static PlacementUpdateHandler instance;
			return &instance;
		}

		RE::BSEventNotifyControl ProcessEvent(const RE::MenuOpenCloseEvent*,
			RE::BSTEventSource<RE::MenuOpenCloseEvent>*)
		{
			if (!g_state.active || !g_state.placedRef)
				return RE::BSEventNotifyControl::kContinue;

			auto input = VRInputHandler::GetSingleton();

			constexpr float angleStep = 2.5f * std::numbers::pi_v<float> / 180.0f;

			bool rightGrip = input->IsRightGripPressed();
			bool leftGrip = input->IsLeftGripPressed();

			if (rightGrip) {
				// Pitch mode (right grip)
				if (input->IsLeftTriggerPressed()) {
					g_state.previewPitch -= angleStep;
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewPitch += angleStep;
				}
			} else if (leftGrip) {
				// Roll mode (left grip)
				if (input->IsLeftTriggerPressed()) {
					g_state.previewRoll -= angleStep;
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewRoll += angleStep;
				}
			} else {
				// Default: yaw mode
				if (input->IsLeftTriggerPressed()) {
					g_state.previewYaw -= angleStep;
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewYaw += angleStep;
				}
			}

			// Distance with right joystick Y
			float joyY = input->GetRightJoystickY();
			if (std::abs(joyY) > 0.1f) {
				g_state.previewDistance += joyY * DISTANCE_STEP;
				g_state.previewDistance = std::clamp(g_state.previewDistance, 50.0f, 3000.0f);
			}

			LivePlace(g_state.placedRef,
				g_state.previewYaw,
				g_state.previewPitch,
				g_state.previewRoll,
				g_state.previewDistance);

			// Confirm placement with A button
			if (input->IsAButtonPressed()) {
				g_state.placedRef->SetMotionType(RE::hkpMotion::MotionType::kDynamic, true);
				RE::DebugNotification("Item Placed");
				OnPlacementConfirmed(g_state.placedRef);
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
			auto player = RE::PlayerCharacter::GetSingleton();
			if (!player || !player->RightWandNode)
				return;

			// --- Position: wand + distance, as you had ---

			auto wandNode = player->RightWandNode.get();
			const auto& wandTransform = wandNode->world;
			const auto& wandPos = wandTransform.translate;

			RE::NiPoint3 localForward{ 0.0f, 0.0f, -1.0f };
			RE::NiPoint3 worldForward = wandTransform.rotate * localForward;

			RE::NiPoint3 targetCenter = wandPos + (worldForward * distance);
			targetCenter.x += g_state.previewXoffset;
			targetCenter.z += g_state.previewZoffset;

			// --- Base frame (from StartLivePlace) ---

			RE::NiPoint3 baseForward = Normalize(g_state.baseForward);
			RE::NiPoint3 baseUp = Normalize(g_state.baseUp);
			RE::NiPoint3 baseRight = Normalize(Cross(baseForward, baseUp));

			// --- Build orientation: yaw -> pitch -> roll with axes that follow ---

			// 1) Yaw around baseUp
			Quaternion qYaw = MakeAxisAngle(baseUp, yawOffset);

			// Axes after yaw
			RE::NiPoint3 yawForward = Normalize(Rotate(qYaw, baseForward));
			RE::NiPoint3 yawRight = Normalize(Rotate(qYaw, baseRight));
			// yawUp is still baseUp (we rotated around baseUp)

			// 2) Pitch around right AFTER yaw
			Quaternion qPitch = MakeAxisAngle(yawRight, pitchOffset);
			Quaternion qYawPitch = Mul(qPitch, qYaw);

			// Axes after yaw + pitch
			RE::NiPoint3 yawPitchForward = Normalize(Rotate(qYawPitch, baseForward));
			RE::NiPoint3 yawPitchUp = Normalize(Rotate(qYawPitch, baseUp));

			// 3) Roll around forward AFTER yaw + pitch
			Quaternion qRoll = MakeAxisAngle(yawPitchForward, rollOffset);
			Quaternion qOrient = Mul(qRoll, qYawPitch);

			// Final basis from qOrient
			RE::NiPoint3 forwardTilted = Normalize(Rotate(qOrient, baseForward));
			RE::NiPoint3 upTilted = Normalize(Rotate(qOrient, baseUp));
			RE::NiPoint3 rightTilted = Normalize(Cross(forwardTilted, upTilted));

			// --- Convert orientation to Skyrim-style Euler (pitch, roll, yaw) ---

			// yaw from forward
			float horizLen = std::sqrt(forwardTilted.x * forwardTilted.x +
									   forwardTilted.y * forwardTilted.y);
			float finalYaw = std::atan2(forwardTilted.x, forwardTilted.y);
			float finalPitch = std::atan2(forwardTilted.z, horizLen);

			// Build "no-roll" up from yaw+pitch only (use qYawPitch)
			RE::NiPoint3 upNoRoll = Normalize(Rotate(qYawPitch, baseUp));

			// Roll is twist of upTilted around forwardTilted
			float finalRoll = SignedAngleAroundAxis(upNoRoll, upTilted, forwardTilted);

			// --- Smooth center + angles ---

			g_state.currentPreviewPos =
				Lerp(g_state.currentPreviewPos, targetCenter, g_state.positionSmoothAlpha);

			float yawDiff = ShortestAngleDiff(g_state.currentPreviewYaw, finalYaw);
			float pitchDiff = ShortestAngleDiff(g_state.currentPreviewPitch, finalPitch);
			float rollDiff = ShortestAngleDiff(g_state.currentPreviewRoll, finalRoll);

			g_state.currentPreviewYaw += yawDiff * g_state.rotationSmoothAlpha;
			g_state.currentPreviewPitch += pitchDiff * g_state.rotationSmoothAlpha;
			g_state.currentPreviewRoll += rollDiff * g_state.rotationSmoothAlpha;

			RE::NiPoint3 appliedCenter = g_state.currentPreviewPos;
			float appliedYaw = g_state.currentPreviewYaw;
			float appliedPitch = g_state.currentPreviewPitch;
			float appliedRoll = g_state.currentPreviewRoll;

			// --- Pivot correction using localCenterOffset (if valid) ---

			RE::NiPoint3 originPos = appliedCenter;
			if (g_state.hasLocalCenterOffset) {
				const RE::NiPoint3 local = g_state.localCenterOffset;

				RE::NiPoint3 offsetWorld{
					rightTilted.x * local.x + upTilted.x * local.y + forwardTilted.x * local.z,
					rightTilted.y * local.x + upTilted.y * local.y + forwardTilted.y * local.z,
					rightTilted.z * local.x + upTilted.z * local.y + forwardTilted.z * local.z
				};

				originPos.x -= offsetWorld.x;
				originPos.y -= offsetWorld.y;
				originPos.z -= offsetWorld.z;
			}

			// DEBUG: visualize center point
			static RE::TESObjectREFR* dbgMarker = nullptr;

			if (!dbgMarker) {
				dbgMarker = SpawnDebugMarkerAt(appliedCenter);
			}

			auto lambdaMarker = dbgMarker;

			// --- Apply on main thread ---

			SKSE::GetTaskInterface()->AddTask([a_refr,
												  originPos,
												  appliedYaw,
												  appliedPitch,
												  appliedRoll,
												  lambdaMarker,
												  appliedCenter] {
				if (!a_refr)
					return;

				// Skyrim: x = pitch, y = roll, z = yaw
				a_refr->SetPosition(originPos);
				a_refr->SetAngle(RE::NiPoint3{ appliedPitch, appliedRoll, appliedYaw });
				a_refr->Update3DPosition(true);
				lambdaMarker->SetPosition(appliedCenter);
				lambdaMarker->Update3DPosition(true);
			});
		}
	};

	void StartLivePlace(RE::TESObjectREFR* placedRef,
		float faceRotation,
		float yMult,
		float zOffset,
		float xOffset)
	{
		if (!placedRef)
			return;

		auto player = RE::PlayerCharacter::GetSingleton();
		if (!player || !player->RightWandNode)
			return;

		// --- Compute center and offset from origin (world space) ---

		auto bounds = BoundsUtil::GetApproxBounds(placedRef);
		RE::NiPoint3 centerWorld = bounds.center;
		RE::NiPoint3 originWorld = placedRef->GetPosition();

		RE::NiPoint3 offsetWorld{
			centerWorld.x - originWorld.x,
			centerWorld.y - originWorld.y,
			centerWorld.z - originWorld.z
		};

		// --- Base frame: face the player horizontally from the center ---

		RE::NiPoint3 playerPos = player->GetPosition();

		RE::NiPoint3 toPlayer{
			playerPos.x - centerWorld.x,
			playerPos.y - centerWorld.y,
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

		RE::NiPoint3 baseForward{
			std::sin(baseYaw),
			std::cos(baseYaw),
			0.0f
		};
		RE::NiPoint3 baseUp{ 0.0f, 0.0f, 1.0f };

		RE::NiPoint3 baseRight = Cross(baseForward, baseUp);
		baseRight = Normalize(baseRight);

		auto dot = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
			return a.x * b.x + a.y * b.y + a.z * b.z;
		};

		g_state.localCenterOffset.x = dot(offsetWorld, baseRight);
		g_state.localCenterOffset.y = dot(offsetWorld, baseUp);
		g_state.localCenterOffset.z = dot(offsetWorld, baseForward);
		g_state.hasLocalCenterOffset = true;

		g_state.baseForward = baseForward;
		g_state.baseUp = baseUp;

		// --- Initial preview center: in front of the wand (not at original ref pos) ---

		auto wandNode = player->RightWandNode.get();
		auto& wandTransform = wandNode->world;
		auto& wandPos = wandTransform.translate;

		RE::NiPoint3 localForward{ 0.0f, 0.0f, -1.0f };
		RE::NiPoint3 worldForward = wandTransform.rotate * localForward;

		// use yMult as a distance override if provided, else use distance from player to original center
		float defaultDist = centerWorld.GetDistance(playerPos);
		float startDist = (yMult > 0.0f) ? yMult : defaultDist;

		RE::NiPoint3 initialCenter = wandPos + (worldForward * startDist);
		initialCenter.x += xOffset;
		initialCenter.z += zOffset + 150.0f;

		// Put physics in keyframed mode during placement
		placedRef->SetMotionType(RE::hkpMotion::MotionType::kKeyframed, false);

		// --- Initialize state ---

		g_state.placedRef = placedRef;
		g_state.active = true;
		g_state.previewYaw = faceRotation * std::numbers::pi_v<float> / 180.0f;
		g_state.previewPitch = 0.0f;
		g_state.previewRoll = 0.0f;

		g_state.previewXoffset = xOffset;
		g_state.previewZoffset = zOffset + 150.0f;
		g_state.previewDistance = startDist;

		g_state.currentPreviewPos = initialCenter;
		g_state.currentPreviewYaw = g_state.previewYaw;
		g_state.currentPreviewPitch = g_state.previewPitch;
		g_state.currentPreviewRoll = g_state.previewRoll;

		// Register for per-frame updates
		if (auto ui = RE::UI::GetSingleton()) {
			ui->AddEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());
		}

		VRInputHandler::Register();
		InputBlocker::EnterEditMode();
		RE::DebugNotification("Live Placement Started");
	}

    void OnPlacementConfirmed(RE::TESObjectREFR* a_refr)
    {
		SKSE::GetTaskInterface()->AddTask([a_refr] { SendModEvent("SBOnPlacementConfirmed", "Done", 0.0f, a_refr); });

		VRInputHandler::UnRegister();
        auto ui = RE::UI::GetSingleton();
        if (ui)
            ui->RemoveEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());

		InputBlocker::ExitEditMode();

		logger::info("Placement Done");
    }

}
