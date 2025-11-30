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
		float previewPitch{ 0.0f };       // desired pitch from input
		float previewRoll{ 0.0f };
        float previewDistance{ 220.0f }; // desired distance from wand

		float previewXoffset{ 0.0f };  // desired X offset from wand
		float previewZoffset{ 0.0f };  // desired X offset from wand

        // baseline and state
        bool hasBaseline{ false };
        RE::NiPoint3 baselineLocal{}; 

		float baseYaw{ 0.0f };
		RE::NiPoint3 baseForward{};
		RE::NiPoint3 baseUp{};

        // current (smoothed) preview transform applied to the object
        RE::NiPoint3 currentPreviewPos{};    // world position currently being applied
        float currentPreviewYaw{ 0.0f };     // yaw (radians) currently being applied
		float currentPreviewPitch{ 0.0f };  // pitch (radians) currently being applied
		float currentPreviewRoll{ 0.0f };

		// New: where is the mesh center relative to the ref origin in local coords
		RE::NiPoint3 localCenterOffset{};
		bool hasLocalCenterOffset{ false };

		RE::NiPoint3 centerOffsetWorld{};
		bool hasCenterOffsetWorld{ false };

        // smoothing factors (0..1). alpha per-frame lerp factor.
        // Recommended starting values: position 0.18, rotation 0.25 — tune as needed.
        float positionSmoothAlpha{ 0.18f };
        float rotationSmoothAlpha{ 0.25f };
    };

    void StartLivePlace(RE::TESObjectREFR* placedRef, float faceRotation, float yMult, float zOffset, float xOffset);
    void OnPlacementConfirmed(RE::TESObjectREFR* a_refr);
}
