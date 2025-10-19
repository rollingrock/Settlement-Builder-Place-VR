#include "VRInputHandler.h"
#include "SKSE/SKSE.h"

VRInputHandler::EventResult VRInputHandler::ProcessEvent(RE::InputEvent* const* a_event, RE::BSTEventSource<RE::InputEvent*>*)
{
    for (auto input = *a_event; input; input = input->next) {
        if (input->eventType == RE::INPUT_EVENT_TYPE::kButton) {
            auto button = static_cast<RE::ButtonEvent*>(input);
            // Log device, key code, and value
            //logger::info(
            //    "[VRInputLogger] Device: {}, IDCode: {}, Value: {}, Held: {}, IsDown: {}",
            //    button->device.underlying(),
            //    button->GetIDCode(),
            //    button->Value(),
            //    button->HeldDuration(),
            //    button->IsDown()
            //);

			// Update isPressed status for a specific button (example: IDCode 1)
			isPressed = isPressed || ((button->device.underlying() == 5) && (button->GetIDCode() == 33) && button->IsPressed());  // Right Hander Trigger
        }
    }
    return RE::BSEventNotifyControl::kContinue;
}

void VRInputHandler::Register()
{
	GetSingleton()->Reset();
    auto inputMgr = RE::BSInputDeviceManager::GetSingleton();
    if (inputMgr) {
        inputMgr->AddEventSink<RE::InputEvent*>(GetSingleton());
        logger::info("[VRInputLogger] Registered input event sink.");
    } else {
        logger::error("[VRInputLogger] Failed to get input manager singleton.");
    }
}

void VRInputHandler::UnRegister()
{
    auto inputMgr = RE::BSInputDeviceManager::GetSingleton();
    if (inputMgr) {
        inputMgr->RemoveEventSink(GetSingleton());
        logger::info("[VRInputLogger] Unregistered input event sink.");
    } else {
        logger::error("[VRInputLogger] Failed to get input manager singleton.");
    }
}

