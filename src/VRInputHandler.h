#pragma once
#include "RE/Skyrim.h"
#include "REX/REX/Singleton.h"
#include <atomic>

class VRInputHandler : public REX::Singleton<VRInputHandler>,
							  RE::BSTEventSink<RE::InputEvent*>
{
public:
    using EventResult = RE::BSEventNotifyControl;

	VRInputHandler() :
		leftTriggerPressed(false), rightTriggerPressed(false), aButtonPressed(false), rightJoystickY(0.0f)
	{}

    EventResult ProcessEvent(RE::InputEvent* const* a_event, RE::BSTEventSource<RE::InputEvent*>* a_eventSource) override;
    static void Register();
    static void UnRegister();

	bool IsLeftTriggerPressed() const;
	bool IsRightTriggerPressed() const;
	bool IsAButtonPressed() const;
	float GetRightJoystickY() const;

	void Reset();

private:
	std::atomic<bool> leftTriggerPressed;
	std::atomic<bool> rightTriggerPressed;
	std::atomic<bool> aButtonPressed;
	std::atomic<float> rightJoystickY;


};

