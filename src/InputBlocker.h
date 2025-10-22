#pragma once
#include "RE/Skyrim.h"

namespace InputBlocker
{
	struct ScopedActionBlock
	{
		bool applied = false;

		void Apply(bool block)
		{
			auto* pc = RE::PlayerControls::GetSingleton();
			if (!pc)
				return;

			for (auto& h : pc->handlers) {
				if (!h)
					continue;

				if (skyrim_cast<RE::JumpHandler*>(h) ||
					skyrim_cast<RE::SneakHandler*>(h)) {
					h->inputEventHandlingEnabled = !block;
					applied = true;
				}
			}
		}
	};

	static ScopedActionBlock g_block;

	void EnterEditMode()
	{
		g_block.Apply(true);  // disable Jump/Sneak handlers
							  // also add your input sink if you're not already
	}

	void ExitEditMode()
	{
		g_block.Apply(false);  // re-enable Jump/Sneak
	}
}
