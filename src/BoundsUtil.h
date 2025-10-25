// BoundsUtil.h
#pragma once
#include "RE/Skyrim.h"
#include <cmath>

namespace BoundsUtil
{
	struct BoundsInfo
	{
		float radius = 0.0f;         // conservative bound sphere radius in world units
		RE::NiPoint3 halfExtents{};  // world-space AABB half-extents
		bool from3D = false;         // true if derived from loaded 3D
	};

	inline BoundsInfo GetApproxBounds(RE::TESObjectREFR* ref)
	{
		BoundsInfo out{};
		if (!ref)
			return out;

		const float scale = ref->GetScale();  // includes refScale & race scale for actors

		// --- 1) Prefer loaded 3D world bound ---
		if (ref->Is3DLoaded()) {
			if (auto* root = ref->Get3D()) {
				// NiAVObject::worldBound is a sphere (center, radius) in WORLD space already
				out.radius = root->worldBound.radius;
				out.from3D = true;

				// If you also want an AABB-ish estimate, map sphere → cube (conservative)
				out.halfExtents = RE::NiPoint3(out.radius, out.radius, out.radius);
				return out;
			}
		}

		// --- 2) Fallback: base form ObjectBounds (local space), scaled to world ---
		// Works for TESBoundObject (STAT/MSTT/ACTI/CONT/WEAP/ARMO/etc.). Non-bound types will return 0s.
		if (auto* base = ref->GetBaseObject()) {
			if (auto* bound = base->As<RE::TESBoundObject>()) {
				const auto& b = bound->boundData;  // min/max in local object space
				// ObjectBounds in CommonLib are typically int16s; promote to float
				RE::NiPoint3 min{ static_cast<float>(b.boundMin.x),
					static_cast<float>(b.boundMin.y),
					static_cast<float>(b.boundMin.z) };
				RE::NiPoint3 max{ static_cast<float>(b.boundMax.x),
					static_cast<float>(b.boundMax.y),
					static_cast<float>(b.boundMax.z) };
				const RE::NiPoint3 extents = (max - min) * 0.5f;
				out.halfExtents = extents * scale;

				// Use max component as a conservative radius; or length() for diagonal
				const float maxComp = (std::max)({ std::fabs(out.halfExtents.x),
					std::fabs(out.halfExtents.y),
					std::fabs(out.halfExtents.z) });
				// Slightly inflate to be safe
				out.radius = maxComp * 1.05f;
			}
		}
		return out;
	}

	// Convenience: compute a spawn position “in front of player” far enough to avoid overlap
	inline RE::NiPoint3 SafeSpawnInFrontOfPlayer(float neededRadius, float extraBuffer = 100.0f)
	{
		auto* pc = RE::PlayerCharacter::GetSingleton();
		const RE::NiPoint3 ppos = pc->GetPosition();

		// Player yaw is in degrees; convert to radians for cos/sin
		const float yawDeg = pc->GetAngleZ();
		const float yawRad = yawDeg * static_cast<float>(std::numbers::pi_v<float> / 180.0);
		const RE::NiPoint3 fwd{ std::cos(yawRad), std::sin(yawRad), 0.0f };

		// Player collision capsule roughly ~35–40 units radius; be generous
		constexpr float kPlayerRadius = 48.0f;

		const float dist = neededRadius + kPlayerRadius + extraBuffer;
		return ppos + fwd * dist;
	}
}
