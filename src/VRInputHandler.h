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
		leftTriggerPressed(false),
		rightTriggerPressed(false),
		aButtonPressed(false),
		rightGripPressed(false),
		leftGripPressed(false),
		rightJoystickY(0.0f),
		leftJoystickButtonPressed(false),
		rightJoystickButtonPressed(false)
	{}

    EventResult ProcessEvent(RE::InputEvent* const* a_event, RE::BSTEventSource<RE::InputEvent*>* a_eventSource) override;

    static void Register();
    static void UnRegister();

	inline bool IsLeftTriggerPressed() const
	{
		return leftTriggerPressed;
	}

	inline bool IsRightTriggerPressed() const
	{
		return rightTriggerPressed;
	}

	inline bool IsRightGripPressed() const
	{
		return rightGripPressed;
	}

	inline bool IsLeftGripPressed() const
	{
		return leftGripPressed;
	}

	inline bool IsAButtonPressed() const
	{
		return aButtonPressed;
	}

	inline float GetRightJoystickY() const
	{
		return rightJoystickY;
	}

	inline bool IsLeftJoystickButtonPressed() const
	{
		return leftJoystickButtonPressed;
	}

	inline bool IsRightJoystickButtonPressed() const
	{
		return rightJoystickButtonPressed;
	}

	void Reset();

private:
	std::atomic<bool> leftTriggerPressed;
	std::atomic<bool> rightTriggerPressed;
	std::atomic<bool> rightGripPressed;
	std::atomic<bool> leftGripPressed;
	std::atomic<bool> aButtonPressed;
	std::atomic<float> rightJoystickY;
	std::atomic<bool> leftJoystickButtonPressed;
	std::atomic<bool> rightJoystickButtonPressed;


};

