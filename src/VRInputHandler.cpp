#include "VRInputHandler.h"
#include "SKSE/SKSE.h"

constexpr int DEVICE_VR_RIGHT = 5;         
constexpr int DEVICE_VR_LEFT = 6;          
constexpr int BUTTON_LEFT_TRIGGER = 33;   
constexpr int BUTTON_RIGHT_TRIGGER = 33;  
constexpr int BUTTON_A = 7;              


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
            //    button->IsHeld()
            //);

			if (button->device.underlying() == DEVICE_VR_LEFT && button->GetIDCode() == BUTTON_LEFT_TRIGGER && button->IsHeld()) {
				leftTriggerPressed = true;
			}
			if (button->device.underlying() == DEVICE_VR_RIGHT && button->GetIDCode() == BUTTON_RIGHT_TRIGGER && button->IsHeld()) {
				rightTriggerPressed = true;
			}
			if (button->device.underlying() == DEVICE_VR_RIGHT && button->GetIDCode() == BUTTON_A && button->IsHeld()) {
				aButtonPressed = true;
			}
        }
		// Detect thumbstick event for right wand
		else if (input->eventType == RE::INPUT_EVENT_TYPE::kThumbstick) {
			auto thumb = static_cast<RE::ThumbstickEvent*>(input);
			if (thumb->device.underlying() == DEVICE_VR_RIGHT) {
				rightJoystickY = thumb->yValue;  
			//	logger::info("[VRInputLogger] Right Joystick Y Value: {}", thumb->yValue);
			}
		}
    }
    return RE::BSEventNotifyControl::kContinue;
}

float VRInputHandler::GetRightJoystickY() const
{
	return rightJoystickY;
}

void VRInputHandler::Register()
{
	GetSingleton()->Reset();
    auto inputMgr = RE::BSInputDeviceManager::GetSingleton();
    if (inputMgr) {
        inputMgr->AddEventSink<RE::InputEvent*>(GetSingleton());
        logger::info("[VRInputHandler] Registered input event sink.");
    } else {
        logger::error("[VRInputHandler] Failed to get input manager singleton.");
    }
}

void VRInputHandler::UnRegister()
{
    auto inputMgr = RE::BSInputDeviceManager::GetSingleton();
    if (inputMgr) {
        inputMgr->RemoveEventSink(GetSingleton());
        logger::info("[VRInputHandler] Unregistered input event sink.");
    } else {
        logger::error("[VRInputHandler] Failed to get input manager singleton.");
    }
	GetSingleton()->Reset();
}

void VRInputHandler::Reset()
{
	leftTriggerPressed = false;
	rightTriggerPressed = false;
	aButtonPressed = false;
	rightJoystickY = 0.0f;
}

bool VRInputHandler::IsLeftTriggerPressed() const
{
	return leftTriggerPressed;
}

bool VRInputHandler::IsRightTriggerPressed() const
{
	return rightTriggerPressed;
}

bool VRInputHandler::IsAButtonPressed() const
{
	return aButtonPressed;
}
