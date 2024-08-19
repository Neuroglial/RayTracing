#include "Material/LambertianMaterial.h"

#include "Utils/Memory.h"
#include "Render/BSDF.h"

namespace RT
{
	AURORA_REGISTER_CLASS(ALambertianMaterial, "Lambertian")

	ALambertianMaterial::ALambertianMaterial(const PropertyTreeNode &node)
	{
		const auto &props = node.getPropertyList();
		Vec3f _kr = props.getVector3f("R");
		Float _tmp[] = { _kr.x, _kr.y, _kr.z };
		m_Kr = Spectrum::fromRGB(_tmp);
		activate();
	}

	void ALambertianMaterial::computeScatteringFunctions(SurfaceInteraction &si, MemoryArena &arena,
		ATransportMode mode, bool allowMultipleLobes) const
	{
		si.bsdf = ARENA_ALLOC(arena, BSDF)(si);
		Spectrum R = m_Kr;
		if (!R.isBlack())
		{
			si.bsdf->add(ARENA_ALLOC(arena, ALambertianReflection)(R));
		}
	}
}