#pragma once

#include "RE/Skyrim.h"

namespace Placement
{
    struct PlacementState
    {
        RE::TESObjectREFR* placedRef{ nullptr };
        bool active{ false };

        // preview control
        float previewYaw{ 0.0f };        // desired yaw from input
        float previewDistance{ 220.0f }; // desired distance from wand

        // baseline and state
        bool hasBaseline{ false };
        RE::NiPoint3 baselineLocal{}; 

        // current (smoothed) preview transform applied to the object
        RE::NiPoint3 currentPreviewPos{};    // world position currently being applied
        float currentPreviewYaw{ 0.0f };     // yaw (radians) currently being applied

        // smoothing factors (0..1). alpha per-frame lerp factor.
        // Recommended starting values: position 0.18, rotation 0.25 — tune as needed.
        float positionSmoothAlpha{ 0.18f };
        float rotationSmoothAlpha{ 0.25f };
    };

    void StartLivePlace(RE::TESObjectREFR* placedRef, float faceRotation, float yMult, float zOffset, float xOffset);
    void OnPlacementConfirmed();
}
