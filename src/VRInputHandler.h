#pragma once
#include "RE/Skyrim.h"
#include "REX/REX/Singleton.h"

class VRInputHandler : public REX::Singleton<VRInputHandler>,
							  RE::BSTEventSink<RE::InputEvent*>
{
public:
    using EventResult = RE::BSEventNotifyControl;

	VRInputHandler() :
		isPressed(false) {};

    EventResult ProcessEvent(RE::InputEvent* const* a_event, RE::BSTEventSource<RE::InputEvent*>* a_eventSource) override;
    static void Register();
    static void UnRegister();

	// Status
	inline bool IsPressed() const { return isPressed; }

	// Reset
	inline void Reset() { isPressed = false;}

private:
	bool isPressed;
};

