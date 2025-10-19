#include "Papyrus.h"
#include "Placement.h"
#include "RE/T/TESObjectREFR.h"

namespace Papyrus
{
	void LivePlace(RE::StaticFunctionTag*, RE::TESObjectREFR* a_refr)
	{
		if (!a_refr) {
			// Handle null reference
			return;
		}

		Placement::StartLivePlace(a_refr);
		return;
	}

	bool RegisterFuncs(RE::BSScript::IVirtualMachine* a_vm)
	{
		if (!a_vm) {
			logger::error("Papyrus - couldn't get VMState");
			return false;
		}

		logger::info("Registering Papyrus functions");

		a_vm->RegisterFunction("LivePlace", "SBPlaceVR"sv, LivePlace);

		logger::info("Papyrus functions registered");
		return true;
	}
}
