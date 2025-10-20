#pragma once
#include "RE/Skyrim.h"

class InputBlocker :
	public RE::BSTEventSink<RE::InputEvent*>
{
public:
	static InputBlocker& GetSingleton()
	{
		static InputBlocker inst;
		return inst;
	}

	void Enable(bool enable)
	{
		if (enable == enabled)
			return;
		enabled = enable;

		auto* mgr = RE::BSInputDeviceManager::GetSingleton();
		if (!mgr)
			return;

		if (enabled) {
			mgr->AddEventSink(this);
		} else {
			mgr->RemoveEventSink(this);
		}
	}

	bool IsEnabled() const { return enabled; }

private:
	InputBlocker() = default;

	using EventResult = RE::BSEventNotifyControl;
	EventResult ProcessEvent(RE::InputEvent* const* a_events,
		RE::BSTEventSource<RE::InputEvent*>*) override
	{
		if (!enabled || !a_events) {
			return EventResult::kContinue;
		}

		auto* user = RE::UserEvents::GetSingleton();
		if (!user) {
			return EventResult::kContinue;
		}

		// Compare against canonical user-event strings
		const char* JUMP = user->jump.c_str();    // "Jump"
		const char* SNEAK = user->sneak.c_str();  // "Sneak"

		for (auto e = *a_events; e; e = e->next) {
			if (const auto* be = e->AsButtonEvent()) {
				// Only suppress actual presses/holds (let releases through, optional)
				if (be->IsPressed() || be->IsHeld()) {
					if (const char* ue = be->QUserEvent().c_str()) {
						logger::info("InputBlocker: Detected user event: {}", ue);
						if ((JUMP && std::strcmp(ue, JUMP) == 0) ||
							(SNEAK && std::strcmp(ue, SNEAK) == 0)) {
							// Swallow this control while edit mode is on
							return EventResult::kStop;  // prevents further propagation
						}
					}
				}
			}
		}

		return EventResult::kContinue;
	}

	std::atomic_bool enabled{ false };
};
