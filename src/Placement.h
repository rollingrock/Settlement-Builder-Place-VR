#pragma once
#include "RE/Skyrim.h"

namespace Placement
{
    struct PlacementState
    {
        RE::TESObjectREFR* placedRef = nullptr;
        bool active = false;
        float previewYaw = 0.0f; // Current yaw for preview
        float previewDistance = 220.0f; // Default starting distance
    };


    void StartLivePlace(RE::TESObjectREFR* placedRef);
    void OnPlacementConfirmed(); // To notify Papyrus
}
