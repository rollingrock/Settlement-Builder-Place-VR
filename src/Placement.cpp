#include "Placement.h"
#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"
#include "VRInputHandler.h"
#include "InputBlocker.h"
#include "BoundsUtil.h"
#include <cmath>


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

	static float NormalizeAngle(float a)
	{
		float d = (a * 180.0f) / std::numbers::pi_v<float>;
		while (d >= 360.0f) d -= 360.0f;
		while (d < 0.0f) d += 360.0f;
		return (d * std::numbers::pi_v<float>) / 180.0f;
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

			// Angle step for both yaw and pitch
			constexpr float angleStep = 2.5f * std::numbers::pi_v<float> / 180.0f;  // 2.5 degrees in radians

			bool rightGrip = input->IsRightGripPressed();
			bool leftGrip = input->IsLeftGripPressed();

			if (rightGrip) {
				// While right grip is held, triggers adjust pitch instead of yaw
				if (input->IsLeftTriggerPressed()) {
					g_state.previewPitch -= angleStep;  // left trigger = pitch down (or invert if you prefer)
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewPitch += angleStep;  // right trigger = pitch up
				}
			} else if (leftGrip) {
				// Roll mode
				if (input->IsLeftTriggerPressed()) {
					g_state.previewRoll -= angleStep;  // roll left
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewRoll += angleStep;  // roll right
				}
			} else {
				// Normal mode: triggers adjust yaw
				if (input->IsLeftTriggerPressed()) {
					g_state.previewYaw -= angleStep;
				}
				if (input->IsRightTriggerPressed()) {
					g_state.previewYaw += angleStep;
				}
			}

			// Adjust distance with right joystick Y
			float joyY = input->GetRightJoystickY();
			if (std::abs(joyY) > 0.1f) {  // Deadzone
				g_state.previewDistance += joyY * DISTANCE_STEP;
				g_state.previewDistance = std::clamp(g_state.previewDistance, 50.0f, 3000.0f);  // Clamp to reasonable range
			}

			// Update preview position and rotation
			LivePlace(g_state.placedRef, g_state.previewYaw, g_state.previewPitch, g_state.previewRoll, g_state.previewDistance);

			// Confirm placement with "A" button
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

		void LivePlace(RE::TESObjectREFR* a_refr, float yawOffset, float pitchOffset, float rollOffset, float distance)
		{
			auto player = RE::PlayerCharacter::GetSingleton();
			if (!player || !player->RightWandNode)
				return;

			// --- Position (same as before, use wand + distance) ---

			auto wandNode = player->RightWandNode.get();
			const auto& wandTransform = wandNode->world;

			const auto& wandPos = wandTransform.translate;

			RE::NiPoint3 localForward{ 0.0f, 0.0f, -1.0f };
			RE::NiPoint3 worldForward = wandTransform.rotate * localForward;

			RE::NiPoint3 targetPos = wandPos + (worldForward * distance);

			targetPos.x += g_state.previewXoffset;
			targetPos.z += g_state.previewZoffset;

			// --- Base yaw: face the player horizontally ---

			const RE::NiPoint3 playerPos = player->GetPosition();

			RE::NiPoint3 toPlayer{
				playerPos.x - targetPos.x,
				playerPos.y - targetPos.y,
				0.0f
			};

			float len2 = toPlayer.x * toPlayer.x + toPlayer.y * toPlayer.y;
			if (len2 < 1e-8f) {
				return;
			}
			float invLen = 1.0f / std::sqrt(len2);
			toPlayer.x *= invLen;
			toPlayer.y *= invLen;
			toPlayer.z = 0.0f;

			float baseYaw = std::atan2(toPlayer.x, toPlayer.y);

			// Combine base yaw with user yaw offset
			float targetYaw = baseYaw + yawOffset;

			// Horizontal forward vector from final yaw (no pitch yet)
			RE::NiPoint3 forwardHoriz{
				std::sin(targetYaw),
				std::cos(targetYaw),
				0.0f
			};
			forwardHoriz = Normalize(forwardHoriz);

			RE::NiPoint3 worldUp{ 0.0f, 0.0f, 1.0f };

			// Right axis for pitch is worldUp x forwardHoriz
			RE::NiPoint3 right = Cross(worldUp, forwardHoriz);
			if (right.x * right.x + right.y * right.y + right.z * right.z < 1e-8f) {
				right = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
			} else {
				right = Normalize(right);
			}

			// --- Quaternion: pitch then roll ---

			Quaternion qPitch = MakeAxisAngle(right, pitchOffset);
			Quaternion qRoll = MakeAxisAngle(forwardHoriz, rollOffset);

			// Order: first pitch, then roll around the new forward
			Quaternion qOrient = Mul(qRoll, qPitch);

			// Rotate forward & up to get the tilted basis
			RE::NiPoint3 forwardTilted = Normalize(Rotate(qOrient, forwardHoriz));
			RE::NiPoint3 upTilted = Normalize(Rotate(qOrient, worldUp));

			RE::NiPoint3 rightTilted = Cross(forwardTilted, upTilted);
			// normalize just in case numeric crap happens
			rightTilted = Normalize(rightTilted);

			// --- Convert orientation to Skyrim-style Euler (pitch, roll, yaw) ---

			// yaw from forward (same convention we used for baseYaw)
			float finalYaw = std::atan2(forwardTilted.x,
				forwardTilted.y);

			// pitch from forward's vertical vs horizontal length
			float horizLen = std::sqrt(forwardTilted.x * forwardTilted.x +
									   forwardTilted.y * forwardTilted.y);
			float finalPitch = std::atan2(forwardTilted.z, horizLen);

			// Build a "no-roll" up vector from yaw + pitch so that we can
			// measure how much upTilted is twisted (roll) around forward.
			RE::NiPoint3 xAxis{ 1.0f, 0.0f, 0.0f };
			RE::NiPoint3 zAxis{ 0.0f, 0.0f, 1.0f };

			Quaternion qYaw = MakeAxisAngle(zAxis, finalYaw);
			Quaternion qPitchNoRoll = MakeAxisAngle(xAxis, finalPitch);
			Quaternion qNoRoll = Mul(qPitchNoRoll, qYaw);

			RE::NiPoint3 upNoRoll = Normalize(Rotate(qNoRoll, worldUp));

			// Roll is the signed angle between upNoRoll and upTilted around forwardTilted
			float finalRoll = SignedAngleAroundAxis(upNoRoll, upTilted, forwardTilted);

			// --- Smooth position and Euler angles ---

			g_state.currentPreviewPos =
				Lerp(g_state.currentPreviewPos, targetPos, g_state.positionSmoothAlpha);

			float yawDiff = ShortestAngleDiff(g_state.currentPreviewYaw, finalYaw);
			float pitchDiff = ShortestAngleDiff(g_state.currentPreviewPitch, finalPitch);
			float rollDiff = ShortestAngleDiff(g_state.currentPreviewRoll, finalRoll);

			g_state.currentPreviewYaw += yawDiff * g_state.rotationSmoothAlpha;
			g_state.currentPreviewPitch += pitchDiff * g_state.rotationSmoothAlpha;
			g_state.currentPreviewRoll += rollDiff * g_state.rotationSmoothAlpha;

			RE::NiPoint3 appliedPos = g_state.currentPreviewPos;
			float appliedYaw = g_state.currentPreviewYaw;
			float appliedPitch = g_state.currentPreviewPitch;
			float appliedRoll = g_state.currentPreviewRoll;
			RE::NiPoint3 appliedCenter = g_state.currentPreviewPos;

			// Default: if we don't know a local center offset, treat origin == center
			RE::NiPoint3 originPos = appliedCenter;

			if (g_state.hasLocalCenterOffset) {
				const RE::NiPoint3 local = g_state.localCenterOffset;

				// Convert local offset back to world using current basis
				RE::NiPoint3 offsetWorld{
					rightTilted.x * local.x + upTilted.x * local.y + forwardTilted.x * local.z,
					rightTilted.y * local.x + upTilted.y * local.y + forwardTilted.y * local.z,
					rightTilted.z * local.x + upTilted.z * local.y + forwardTilted.z * local.z
				};

				originPos.x = appliedCenter.x - offsetWorld.x;
				originPos.y = appliedCenter.y - offsetWorld.y;
				originPos.z = appliedCenter.z - offsetWorld.z;
			}

			// --- Apply on main thread ---

			SKSE::GetTaskInterface()->AddTask([a_refr,
												  originPos,
												  appliedYaw,
												  appliedPitch,
												  appliedRoll] {
				if (!a_refr)
					return;

				a_refr->SetPosition(originPos);
				// Skyrim: x = pitch, y = roll, z = yaw
				a_refr->SetAngle(RE::NiPoint3{ appliedPitch, appliedRoll, appliedYaw });
				a_refr->Update3DPosition(true);
			});
		}
	};

    void StartLivePlace(RE::TESObjectREFR* placedRef, float faceRotation, float yMult, float zOffset, float xOffset)
    {
        if (!placedRef)
            return;

		auto bounds = BoundsUtil::GetApproxBounds(placedRef);  // your existing helper
		RE::NiPoint3 centerWorld = bounds.center;              // adjust if your struct differs

		RE::NiPoint3 originWorld = placedRef->GetPosition();
		RE::NiPoint3 offsetWorld{
			centerWorld.x - originWorld.x,
			centerWorld.y - originWorld.y,
			centerWorld.z - originWorld.z
		};

		// Build local basis from the ref's current angles
		RE::NiPoint3 ang = placedRef->GetAngle();  // x=pitch, y=roll, z=yaw
		float pitch = ang.x;
		float roll = ang.y;
		float yaw = ang.z;

		float cp = std::cos(pitch), sp = std::sin(pitch);
		float cr = std::cos(roll), sr = std::sin(roll);
		float cy = std::cos(yaw), sy = std::sin(yaw);

		// One conventional Skyrim basis (match your LivePlace convention!)
		RE::NiPoint3 forward{
			sy * cp,
			cy * cp,
			sp
		};

		RE::NiPoint3 right{
			cy * cr + sy * sp * sr,
			-sy * cr + cy * sp * sr,
			-cp * sr
		};

		RE::NiPoint3 up{
			cy * sr - sy * sp * cr,
			sy * sr + cy * sp * cr,
			cp * cr
		};

		auto dot = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
			return a.x * b.x + a.y * b.y + a.z * b.z;
		};

		// Inverse of an orthonormal basis is just the transpose:
		RE::NiPoint3 localOffset{
			dot(offsetWorld, right),
			dot(offsetWorld, up),
			dot(offsetWorld, forward)
		};

		g_state.localCenterOffset = localOffset;
		g_state.hasLocalCenterOffset = true;

		// Set the placedRef to kKeyframed so physics doesn't interfere during placement)
		placedRef->SetMotionType(RE::hkpMotion::MotionType::kKeyframed, false);

        g_state.placedRef = placedRef;
        g_state.active = true;
        g_state.previewYaw = faceRotation * std::numbers::pi_v<float> / 180.0f; 
		g_state.previewPitch = 0.0f;  // start level
		g_state.previewRoll = 0.0f;
		
        // Reset smoothing state so preview starts from current ref transform
		g_state.previewXoffset = xOffset;
		g_state.previewZoffset = zOffset + 150.0f;
		g_state.currentPreviewPos = centerWorld;
		g_state.currentPreviewYaw = g_state.previewYaw;
		g_state.currentPreviewPitch = g_state.previewPitch;
		g_state.currentPreviewRoll = g_state.previewRoll;

		auto distance = centerWorld.GetDistance(RE::PlayerCharacter::GetSingleton()->GetPosition());
		g_state.previewDistance = yMult > 0.0 ? yMult : distance;


        // Register for per-frame updates (using MenuOpenCloseEvent as a simple per-frame hook)
        auto ui = RE::UI::GetSingleton();
        if (ui)
            ui->AddEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());

		VRInputHandler::Register();
		InputBlocker::EnterEditMode();
		RE::DebugNotification("Live Placement Started");
		logger::info("Live Placement Started for ref: {} {}", placedRef->GetFormID(), distance);
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
