#include "Utils/Interaction.h"

#include "Utils/Color.h"
#include "Object/Hitable.h"
#include "Render/BSDF.h"

namespace RT
{
	SurfaceInteraction::SurfaceInteraction(const Vec3f &p, const Vec2f &uv, const Vec3f &wo,
		const Vec3f &dpdu, const Vec3f &dpdv, const Shape *sh)
		: Interaction(p, normalize(cross(dpdu, dpdv)), wo), uv(uv), dpdu(dpdu), dpdv(dpdv), shape(sh) {}

	Spectrum SurfaceInteraction::Le(const Vec3f &w) const
	{
		const AreaLight *area = hitable->getAreaLight();
		return area != nullptr ? area->L(*this, w) : Spectrum(0.f);
	}

	void SurfaceInteraction::computeScatteringFunctions(const Ray &ray, MemoryArena &arena,
		bool allowMultipleLobes, TransportMode mode)
	{
		hitable->computeScatteringFunctions(*this, arena, mode, allowMultipleLobes);
	}

}