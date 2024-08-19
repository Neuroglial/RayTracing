#ifndef ARMIRRORMATERIAL_H
#define ARMIRRORMATERIAL_H

#include "Material/Material.h"

namespace RT
{
	class AMirrorMaterial final : public Material
	{
	public:
		typedef std::shared_ptr<AMirrorMaterial> ptr;

		AMirrorMaterial(const PropertyTreeNode &node);
		AMirrorMaterial(const Spectrum &r) : m_Kr(r) {}

		virtual void computeScatteringFunctions(SurfaceInteraction &si, MemoryArena &arena,
			ATransportMode mode, bool allowMultipleLobes) const override;

		virtual std::string toString() const override { return "MirrorMaterial[]"; }

	private:
		Spectrum m_Kr;
	};
}

#endif