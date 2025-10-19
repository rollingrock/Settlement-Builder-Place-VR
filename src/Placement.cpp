#include "Placement.h"
#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"
#include "VRInputHandler.h"

namespace Placement
{
    static PlacementState g_state;

	inline void SendModEvent(std::string_view eventName, std::string_view strArg = "", float numArg = 0.0f, RE::TESForm* sender = nullptr)
	{
		// BSFixedString can be built from const char*
		SKSE::ModCallbackEvent e{ eventName.data(), strArg.data(), numArg, sender };
		SKSE::GetModCallbackEventSource()->SendEvent(&e);
	}

	inline float NormalizeAngle(float a)
	{
		float d = (a * 180.0f) / std::numbers::pi_v<float>;
		while (d >= 360.0f) d -= 360.0f;
		while (d < 0.0f) d += 360.0f;
		return (d * std::numbers::pi_v<float>) / 180.0f;
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

            // Per-frame logic here (update preview, check input)
            auto player = RE::PlayerCharacter::GetSingleton();
            if (!player)
                return RE::BSEventNotifyControl::kContinue;

			LivePlace(g_state.placedRef);

            if (VRInputHandler::GetSingleton()->IsPressed()) {
                // Finalize placement
                g_state.placedRef->SetMotionType(RE::hkpMotion::MotionType::kDynamic, true);
                RE::DebugNotification("Item Placed");
                g_state.active = false;
				g_state.placedRef = nullptr;
				g_state.hasBaseline = false;
                OnPlacementConfirmed(); // Notify Papyrus
            }


            return RE::BSEventNotifyControl::kContinue;
        }

		void LivePlace(RE::TESObjectREFR* a_refr) {
			auto player = RE::PlayerCharacter::GetSingleton();
			if (!player || !player->RightWandNode) return;

			// Get the world transform of the right wand node
			auto wandNode = player->RightWandNode.get();
			const auto& wandTransform = wandNode->world;

			// The position of the wand in world space
			const auto& wandPos = wandTransform.translate;

			// The forward direction of the wand in world space
			// In Skyrim, forward is usually the -Z axis in local space
			RE::NiPoint3 localForward{ 0.0f, 0.0f, -1.0f };
			RE::NiPoint3 worldForward = wandTransform.rotate * localForward;

			// Calculate the target position 100 units in front of the wand
			float distance = 220.0f;
			RE::NiPoint3 targetPos = wandPos + (worldForward * distance);

			// Convert the wand's rotation matrix to Euler angles (in radians)
			// Skyrim's SetAngle expects angles in radians
			float pitch, yaw, roll;
			// Extract Euler angles from the rotation matrix
			// Assuming the matrix is in row-major order:
			// yaw   (Z) = atan2(m10, m00)
			// pitch (Y) = atan2(-m20, sqrt(m21^2 + m22^2))
			// roll  (X) = atan2(m21, m22)
			const auto& m = wandTransform.rotate;
			yaw   = std::atan2(m.entry[1][0], m.entry[0][0]);
			pitch = std::atan2(-m.entry[2][0], std::sqrt(m.entry[2][1] * m.entry[2][1] + m.entry[2][2] * m.entry[2][2]));
			roll  = std::atan2(m.entry[2][1], m.entry[2][2]);

			yaw = player->GetAngleZ();

			SKSE::GetTaskInterface()->AddTask([a_refr, targetPos, pitch, roll, yaw] {
				a_refr->SetPosition(targetPos);
				a_refr->SetAngle(RE::NiPoint3{ 0.0f, 0.0f, NormalizeAngle(yaw) });  // X=roll, Y=pitch, Z=yaw
				a_refr->Update3DPosition(true);
			});
		}
    };

    void StartLivePlace(RE::TESObjectREFR* placedRef)
    {
        if (!placedRef)
            return;

        g_state.placedRef = placedRef;
        g_state.active = true;
		g_state.hasBaseline = false;

        // Set up preview state, etc.

        // Register for per-frame updates (using MenuOpenCloseEvent as a simple per-frame hook)
        auto ui = RE::UI::GetSingleton();
        if (ui)
            ui->AddEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());

		VRInputHandler::Register();
		RE::DebugNotification("Live Placement Started");
		logger::info("Live Placement Started for ref: {}", placedRef->GetFormID());
    }

    void OnPlacementConfirmed()
    {
		SKSE::GetTaskInterface()->AddTask([] { SendModEvent("SBOnPlacementConfirmed", "Done", 0.0f, nullptr); });

		VRInputHandler::UnRegister();
        auto ui = RE::UI::GetSingleton();
        if (ui)
            ui->RemoveEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());


		logger::info("Placement Done");
    }

}
