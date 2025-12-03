#pragma once

#include "RE/Skyrim.h"

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

		float previewXoffset{ 0.0f };  // world X offset from wand
		float previewZoffset{ 0.0f };  // world Z offset from wand

		// Base frame (initial facing from "pivot" toward player)
		float baseYaw{ 0.0f };
		RE::NiPoint3 baseForward{};
		RE::NiPoint3 baseUp{};

		// Preview state
		RE::NiPoint3 currentPreviewPos{};  // world-space pivot/center position being applied

		// Pivot node and offsets (for child-geometry case)
		RE::NiPointer<RE::NiAVObject> pivotNode;
		RE::NiPoint3 pivotOffsetWorld{};  // world-space offset root -> pivot
		bool hasPivot{ false };

		RE::NiMatrix3 pivotOriginalRotate{};  // original local rotate of pivot node
		bool hasPivotOriginalRotate{ false };

		// Whether this ref is a "root-only geometry" case: use SetAngle on the root
		bool useRootAngle{ false };

		// Smoothing factors (0..1)
		float positionSmoothAlpha{ 0.18f };
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
