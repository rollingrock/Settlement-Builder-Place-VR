#include "Papyrus.h"
#include "Placement.h"
#include "RE/T/TESObjectREFR.h"

namespace Papyrus
{
	void LivePlace(RE::StaticFunctionTag*,
		RE::TESObjectREFR* a_refr,
		float faceRotation,
		float yMult,
		float zOffset,
		float xOffset)
	{
		if (!a_refr) {
			return;
		}

		Placement::StartLivePlace(a_refr, faceRotation, yMult, zOffset, xOffset);
	}

	bool ApplyPreviewTransforms(RE::StaticFunctionTag*,
		RE::TESObjectREFR* previewRef,
		RE::TESObjectREFR* finalRef)
	{
		if (!previewRef || !finalRef) {
			return false;
		}

		return Placement::ApplyPreviewTransforms(previewRef, finalRef);
	}

	bool RegisterFuncs(RE::BSScript::IVirtualMachine* a_vm)
	{
		if (!a_vm) {
			logger::error("Papyrus - couldn't get VMState");
			return false;
		}

		logger::info("Registering Papyrus functions");

		a_vm->RegisterFunction("LivePlace", "SBPlaceVR"sv, LivePlace);
		a_vm->RegisterFunction("ApplyPreviewTransforms", "SBPlaceVR"sv, ApplyPreviewTransforms);

		logger::info("Papyrus functions registered");
		return true;
	}
}
