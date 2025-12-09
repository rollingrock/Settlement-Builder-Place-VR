#pragma once

#include "RE/Skyrim.h"
#include <chrono>

namespace Placement
{
	struct PlacementState
	{
		RE::TESObjectREFR* placedRef{ nullptr };
		bool active{ false };

		// Preview control (user intent)
		float previewYaw{ 0.0f };         // radians, user yaw offset
		float previewPitch{ 0.0f };       // radians, user pitch offset
		float previewRoll{ 0.0f };        // radians, user roll offset
		float previewDistance{ 220.0f };  // desired distance from wand

		// Smoothed rotation state
		float currentYaw{ 0.0f };         // smoothed yaw being applied
		float currentPitch{ 0.0f };       // smoothed pitch being applied
		float currentRoll{ 0.0f };        // smoothed roll being applied

		float previewXoffset{ 0.0f };  // world X offset from wand
		float previewZoffset{ 0.0f };  // world Z offset from wand

		// Base frame (initial facing from "pivot" toward player)
		float baseYaw{ 0.0f };
		RE::NiPoint3 baseForward{};
		RE::NiPoint3 baseUp{};
		RE::NiPoint3 baseRight{};
		bool hasBaseFrame{ false };

		// Wand orientation tracking (for rotation lock)
		float initialWandYaw{ 0.0f };  // Initial horizontal yaw of wand at placement start

		// Preview state
		RE::NiPoint3 currentPreviewPos{};  // world-space pivot/center position being applied

		// Pivot node and offsets (for child-geometry case)
		RE::NiPointer<RE::NiAVObject> pivotNode;
		RE::NiPoint3 pivotOffsetWorld{};  // world-space offset root -> pivot
		bool hasPivot{ false };

		RE::NiMatrix3 pivotOriginalRotate{};  // original local rotate of pivot node
		bool hasPivotOriginalRotate{ false };

		RE::NiMatrix3 rootWorldRotate{};  // root node's world rotation (for transforming to local space)
		bool hasRootWorldRotate{ false };

		RE::NiMatrix3 rootOriginalLocalRotate{};  // root node's original local rotation
		bool hasRootOriginalLocalRotate{ false };

		// Whether this ref is a "root-only geometry" case: use SetAngle on the root
		bool useRootAngle{ false };

		// Smoothing factors (0..1)
		float positionSmoothAlpha{ 0.18f };
		float rotationSmoothAlpha{ 0.25f };  // 0.25 = balanced smoothness

		// Initial rotation values for reset functionality
		float initialYaw{ 0.0f };
		float initialPitch{ 0.0f };
		float initialRoll{ 0.0f };
		bool hasInitialRotation{ false };

		// Reset button hold timing
		std::chrono::steady_clock::time_point resetButtonsFirstPressed;
		bool resetButtonsBeingHeld{ false };
		const float resetHoldDurationMs{ 50.0f };  // Minimal delay to prevent accidents
	};

	void StartLivePlace(RE::TESObjectREFR* placedRef,
		float faceRotationDeg,
		float yMult,
		float zOffset,
		float xOffset);

	void OnPlacementConfirmed(RE::TESObjectREFR* a_refr);

	// Called from Papyrus after the final ref is spawned.
	// For child-geometry objects: copy the preview's pivot-node local.rotate onto the final ref.
	// For root-only geometry objects: just copy the root position/angle.
	bool ApplyPreviewTransforms(RE::TESObjectREFR* previewRef,
		RE::TESObjectREFR* finalRef);
}
