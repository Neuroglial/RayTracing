#pragma once

#include "Material/Material.h"

namespace RT
{
	class ALambertianMaterial final : public Material
	{
	public:
		typedef std::shared_ptr<ALambertianMaterial> ptr;

		ALambertianMaterial(const PropertyTreeNode &node);
		ALambertianMaterial(const Spectrum &r) : m_Kr(r) {}

		virtual void computeScatteringFunctions(SurfaceInteraction &si, MemoryArena &arena,
			ATransportMode mode, bool allowMultipleLobes) const override;

		virtual std::string toString() const override { return "LambertianMaterial[]"; }

	private:
		Spectrum m_Kr;
	};
}