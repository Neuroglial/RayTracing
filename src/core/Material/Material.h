#ifndef ARMATERIAL_H
#define ARMATERIAL_H

#include "Utils/Base.h"
#include "Utils/Color.h"
#include "Utils/Math.h"
#include "Object/Object.h"

namespace RT
{

	class Material : public Object
	{
	public:
		typedef std::shared_ptr<Material> ptr;

		Material() = default;
		virtual ~Material() = default;

		virtual void computeScatteringFunctions(SurfaceInteraction &si, MemoryArena &arena,
			ATransportMode mode, bool allowMultipleLobes) const = 0;

		virtual AClassType getClassType() const override { return AClassType::AEMaterial; }

	};

}

#endif