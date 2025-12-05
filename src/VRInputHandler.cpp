#include "VRInputHandler.h"
#include "SKSE/SKSE.h"

constexpr int DEVICE_VR_RIGHT = 5;
constexpr int DEVICE_VR_LEFT = 6;
constexpr int BUTTON_LEFT_TRIGGER = 33;
constexpr int BUTTON_RIGHT_TRIGGER = 33;
constexpr int BUTTON_A = 7;
constexpr int BUTTON_RIGHT_GRIP = 2;
constexpr int BUTTON_LEFT_GRIP = 2;
constexpr int BUTTON_LEFT_JOYSTICK = 32;
constexpr int BUTTON_RIGHT_JOYSTICK = 32;

VRInputHandler::EventResult VRInputHandler::ProcessEvent(RE::InputEvent* const* a_event, RE::BSTEventSource<RE::InputEvent*>*)
{
    for (auto input = *a_event; input; input = input->next) {
        if (input->eventType == RE::INPUT_EVENT_TYPE::kButton) {
            auto button = static_cast<RE::ButtonEvent*>(input);

			// Left trigger
			if (button->device.underlying() == DEVICE_VR_LEFT &&
				button->GetIDCode() == BUTTON_LEFT_TRIGGER &&
				button->IsHeld()) {
				leftTriggerPressed = true;
			}

			// Right trigger
			if (button->device.underlying() == DEVICE_VR_RIGHT &&
				button->GetIDCode() == BUTTON_RIGHT_TRIGGER &&
				button->IsHeld()) {
				rightTriggerPressed = true;
			}

			// A button
			if (button->device.underlying() == DEVICE_VR_RIGHT &&
				button->GetIDCode() == BUTTON_A &&
				button->IsHeld()) {
				aButtonPressed = true;
			}

			// Right grip (held)
			if (button->device.underlying() == DEVICE_VR_RIGHT &&
				button->GetIDCode() == BUTTON_RIGHT_GRIP &&
				button->IsHeld()) {
				rightGripPressed = true;
			}
			// Left grip (held)
			if (button->device.underlying() == DEVICE_VR_LEFT &&
				button->GetIDCode() == BUTTON_LEFT_GRIP &&
				button->IsHeld()) {
				leftGripPressed = true;
			}

		// Left joystick button
		if (button->device.underlying() == DEVICE_VR_LEFT &&
			button->GetIDCode() == BUTTON_LEFT_JOYSTICK &&
			button->IsHeld()) {
			leftJoystickButtonPressed = true;
		}

		// Right joystick button
		if (button->device.underlying() == DEVICE_VR_RIGHT &&
			button->GetIDCode() == BUTTON_RIGHT_JOYSTICK &&
			button->IsHeld()) {
			rightJoystickButtonPressed = true;
		}
        }
		// Detect thumbstick event for right wand
		else if (input->eventType == RE::INPUT_EVENT_TYPE::kThumbstick) {
			auto thumb = static_cast<RE::ThumbstickEvent*>(input);
			if (thumb->device.underlying() == DEVICE_VR_RIGHT) {
				rightJoystickY = thumb->yValue;
			}
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
	rightGripPressed = false;
	leftGripPressed = false;
	rightJoystickY = 0.0f;
	leftJoystickButtonPressed = false;
	rightJoystickButtonPressed = false;
}

