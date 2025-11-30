#pragma once
#include "RE/Skyrim.h"
#include <cmath>

	namespace BoundsUtil
{
	struct BoundsInfo
	{
		float radius = 0.0f;         // conservative bound sphere radius in world units
		RE::NiPoint3 halfExtents{};  // world-space AABB-ish half-extents
		bool from3D = false;         // true if derived from loaded 3D
		RE::NiPoint3 center{};       // WORLD-space center of the bounds
	};

	inline BoundsInfo GetApproxBounds(RE::TESObjectREFR * ref)
	{
		BoundsInfo out{};
		if (!ref) {
			return out;
		}

		const float scale = ref->GetScale();  // includes refScale & race scale for actors

		// --- 1) Prefer loaded 3D world bound ---
		if (ref->Is3DLoaded()) {
			if (auto* root = ref->Get3D()) {
				// worldBound is already in WORLD space
				out.radius = root->worldBound.radius;
				out.center = root->worldBound.center;
				out.from3D = true;
				out.halfExtents = RE::NiPoint3(out.radius, out.radius, out.radius);
				return out;
			}
		}

		// --- 2) Fallback: base form ObjectBounds (local space), scaled to world ---
		// For placement we mostly care about radius; center is set to ref position as a safe default.
		out.center = ref->GetPosition();

		if (auto* base = ref->GetBaseObject()) {
			if (auto* bound = base->As<RE::TESBoundObject>()) {
				const auto& b = bound->boundData;  // min/max in local object space

				RE::NiPoint3 min{
					static_cast<float>(b.boundMin.x),
					static_cast<float>(b.boundMin.y),
					static_cast<float>(b.boundMin.z)
				};
				RE::NiPoint3 max{
					static_cast<float>(b.boundMax.x),
					static_cast<float>(b.boundMax.y),
					static_cast<float>(b.boundMax.z)
				};

				const RE::NiPoint3 extents = (max - min) * 0.5f;
				out.halfExtents = extents * scale;

				const float maxComp = (std::max)({ std::fabs(out.halfExtents.x),
					std::fabs(out.halfExtents.y),
					std::fabs(out.halfExtents.z) });

				out.radius = maxComp * 1.05f;  // conservative inflate
			}
		}

		return out;
	}

	// Convenience: compute a spawn position “in front of player” far enough to avoid overlap
	inline RE::NiPoint3 SafeSpawnInFrontOfPlayer(float neededRadius, float extraBuffer = 100.0f)
	{
		auto* pc = RE::PlayerCharacter::GetSingleton();
		const RE::NiPoint3 ppos = pc->GetPosition();

		const float yawDeg = pc->GetAngleZ();
		const float yawRad = yawDeg * static_cast<float>(std::numbers::pi_v<float> / 180.0);
		const RE::NiPoint3 fwd{ std::cos(yawRad), std::sin(yawRad), 0.0f };

		constexpr float kPlayerRadius = 48.0f;
		const float dist = neededRadius + kPlayerRadius + extraBuffer;
		return ppos + fwd * dist;
	}
}
