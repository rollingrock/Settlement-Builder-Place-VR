#include "Placement.h"
#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"
#include "VRInputHandler.h"
#include "InputBlocker.h"

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

            // Adjust yaw with triggers
            constexpr float yawStep = 2.5f * std::numbers::pi_v<float> / 180.0f; // 2.5 degrees in radians
            if (input->IsLeftTriggerPressed()) {
                g_state.previewYaw -= yawStep;
            }
            if (input->IsRightTriggerPressed()) {
                g_state.previewYaw += yawStep;
            }

            // Adjust distance with right joystick Y
            float joyY = input->GetRightJoystickY();
            if (std::abs(joyY) > 0.1f) { // Deadzone
                g_state.previewDistance += joyY * DISTANCE_STEP;
                g_state.previewDistance = std::clamp(g_state.previewDistance, 50.0f, 1000.0f); // Clamp to reasonable range
            }

            // Update preview position and rotation
            LivePlace(g_state.placedRef, g_state.previewYaw, g_state.previewDistance);

            // Confirm placement with "A" button
            if (input->IsAButtonPressed()) {
                g_state.placedRef->SetMotionType(RE::hkpMotion::MotionType::kDynamic, true);
                RE::DebugNotification("Item Placed");
                g_state.active = false;
                g_state.placedRef = nullptr;
                OnPlacementConfirmed();
            }

			input->Reset();

            return RE::BSEventNotifyControl::kContinue;
        }

		void LivePlace(RE::TESObjectREFR* a_refr, float yaw, float distance) {
			auto player = RE::PlayerCharacter::GetSingleton();
			if (!player || !player->RightWandNode) return;

			// Get the world transform of the right wand node
			auto wandNode = player->RightWandNode.get();
			const auto& wandTransform = wandNode->world;

			// The position of the wand in world space
			const auto& wandPos = wandTransform.translate;

			// The forward direction of the wand in world space
			RE::NiPoint3 localForward{ 0.0f, 0.0f, -1.0f };
			RE::NiPoint3 worldForward = wandTransform.rotate * localForward;

			RE::NiPoint3 targetPos = wandPos + (worldForward * distance);

			SKSE::GetTaskInterface()->AddTask([a_refr, targetPos, yaw] {
				a_refr->SetPosition(targetPos);
				a_refr->SetAngle(RE::NiPoint3{ 0.0f, 0.0f, yaw });
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
        g_state.previewYaw = 0.0f; 

        // Register for per-frame updates (using MenuOpenCloseEvent as a simple per-frame hook)
        auto ui = RE::UI::GetSingleton();
        if (ui)
            ui->AddEventSink<RE::MenuOpenCloseEvent>(PlacementUpdateHandler::GetSingleton());

		VRInputHandler::Register();
		//InputBlocker::GetSingleton().Enable(true);
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

		//InputBlocker::GetSingleton().Enable(false);

		logger::info("Placement Done");
    }

}
