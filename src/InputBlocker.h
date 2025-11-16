//#pragma once
//#include "RE/Skyrim.h"
//
//namespace InputBlocker
//{
//	struct ScopedActionBlock
//	{
//		bool applied = false;
//
//		void Apply(bool block)
//		{
//			auto* pc = RE::PlayerControls::GetSingleton();
//			if (!pc)
//				return;
//
//			for (auto& h : pc->handlers) {
//				if (!h)
//					continue;
//
//				if (skyrim_cast<RE::JumpHandler*>(h) ||
//					skyrim_cast<RE::SneakHandler*>(h)) {
//					h->inputEventHandlingEnabled = !block;
//					applied = true;
//				}
//			}
//		}
//	};
//
//	static ScopedActionBlock g_block;
//
//	void EnterEditMode()
//	{
//		g_block.Apply(true);  // disable Jump/Sneak handlers
//							  // also add your input sink if you're not already
//	}
//
//	void ExitEditMode()
//	{
//		g_block.Apply(false);  // re-enable Jump/Sneak
//	}
//}

#pragma once
#include "RE/Skyrim.h"
#include <unordered_map>

namespace InputBlocker
{
	class ScopedActionBlock final :
		public RE::BSTEventSink<RE::InputEvent*>
	{
	public:
		void Apply(bool block)
		{
			auto* pc = RE::PlayerControls::GetSingleton();
			if (!pc)
				return;

			// handlers can be rebuilt on load; clear stale cache when we see a new set
			if (lastHandlersPtr != pc->handlers.data()) {
				lastHandlersPtr = pc->handlers.data();
				original.clear();
			}

			for (auto& h : pc->handlers) {
				if (!h)
					continue;

				// VR headers: handlers are raw pointers, not smart ptrs
				if (skyrim_cast<RE::JumpHandler*>(h) || skyrim_cast<RE::SneakHandler*>(h)) {
					auto it = original.find(h);
					if (it == original.end()) {
						original[h] = h->inputEventHandlingEnabled;  // remember actual prior value
					}
					h->inputEventHandlingEnabled = !block;
				}
			}

			if (block) {
				if (++refCount == 1) {
					// optional: add yourself as an input sink if you also read axes
					RE::BSInputDeviceManager::GetSingleton()->AddEventSink(this);
				}
			} else {
				if (refCount && --refCount == 0) {
					// restore every tracked handler to its remembered state
					for (auto& [ptr, wasEnabled] : original) {
						if (ptr)
							ptr->inputEventHandlingEnabled = wasEnabled;
					}
					original.clear();
					RE::BSInputDeviceManager::GetSingleton()->RemoveEventSink(this);
				}
			}
		}

		// noop sink (only here if you also want to swallow axes/buttons when active)
		using EventResult = RE::BSEventNotifyControl;
		EventResult ProcessEvent(RE::InputEvent* const*, RE::BSTEventSource<RE::InputEvent*>*) override
		{
			return refCount ? EventResult::kStop : EventResult::kContinue;
		}

	private:
		std::unordered_map<RE::PlayerInputHandler*, bool> original;
		const void* lastHandlersPtr{ nullptr };
		uint32_t refCount{ 0 };
	};

	static ScopedActionBlock g_block;

	inline void EnterEditMode() { g_block.Apply(true); }
	inline void ExitEditMode() { g_block.Apply(false); }
}
