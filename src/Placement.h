#pragma once
#include "RE/Skyrim.h"

namespace Placement
{
    struct PlacementState
    {
        RE::TESObjectREFR* placedRef = nullptr;
        bool active = false;
        // Add more fields as needed (baseline, etc.)
		RE::NiPoint3 baselineLocal = { 0.0, 0.0, 0.0 };
		bool hasBaseline = false;
    };


    void StartLivePlace(RE::TESObjectREFR* placedRef);
    void OnPlacementConfirmed(); // To notify Papyrus
}
