#ifndef ARINTERACTION_H
#define ARINTERACTION_H

#include "Utils/Base.h"
#include "Utils/Math.h"

namespace RT
{
	class Interaction
	{
	public:

		Interaction() = default;
		Interaction(const Vec3f &p) : p(p) {}
		Interaction(const Vec3f &p, const Vec3f &wo) : p(p), wo(normalize(wo)) {}
		Interaction(const Vec3f &p, const Vec3f &n, const Vec3f &wo)
			: p(p), wo(normalize(wo)), n(n) {}

		//从碰撞点发出与d同向光线
		inline Ray spawnRay(const Vec3f &d) const
		{
			Vec3f o = p;
			return Ray(o, d, Infinity);
		}

		//从碰撞点发出与到p点的光线
		inline Ray spawnRayTo(const Vec3f &p2) const
		{
			Vec3f origin = p;
			return Ray(origin, p2 - p, 1 - ShadowEpsilon);
		}

		// 从碰撞点发出与到另一个碰撞点点的光线
		inline Ray spawnRayTo(const Interaction &it) const
		{
			Vec3f origin = p;
			Vec3f target = it.p;
			Vec3f d = target - origin;
			return Ray(origin, d, 1 - ShadowEpsilon);
		}

	public:
		Vec3f p;			//surface point
		Vec3f wo;			//outgoing direction
		Vec3f n;			//normal vector
	};

	class SurfaceInteraction final : public Interaction
	{
	public:

		SurfaceInteraction() = default;
		SurfaceInteraction(const Vec3f &p, const Vec2f &uv, const Vec3f &wo,
			const Vec3f &dpdu, const Vec3f &dpdv, const Shape *sh);

		Spectrum Le(const Vec3f &w) const;

		void computeScatteringFunctions(const Ray &ray, MemoryArena &arena,
			bool allowMultipleLobes = false, TransportMode mode = TransportMode::Radiance);

	public:
		Vec2f uv;
		Vec3f dpdu, dpdv;

		BSDF* bsdf = nullptr;

		const Shape *shape = nullptr;
		const Hitable *hitable = nullptr;

	};
}

#endif