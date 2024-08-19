#include "Shape/Shape.h"

#include "Utils/Interaction.h"
#include "Render/Sampler.h"

namespace RT
{
	Shape::Shape(const PropertyList &props) {}

	Shape::Shape(Transform *objectToWorld, Transform *worldToObject)
		: m_objectToWorld(objectToWorld), m_worldToObject(worldToObject) {}

	bool Shape::hit(const Ray &ray) const
	{
		Float tHit = ray.m_tMax;
		SurfaceInteraction isect;
		return hit(ray, tHit, isect);
	}

	void Shape::setTransform(Transform *objectToWorld, Transform *worldToObject)
	{
		m_objectToWorld = objectToWorld;
		m_worldToObject = worldToObject;
	}

	BBox3f Shape::worldBound() const { return (*m_objectToWorld)(objectBound()); }

	Interaction Shape::sample(const Interaction &ref, const Vec2f &u, Float &pdf) const
	{
		// Sample a point on the shape given a reference point |ref| and
		// return the PDF with respect to solid angle from |ref|.
		Interaction intr = sample(u, pdf);
		Vec3f wi = intr.p - ref.p;
		if (dot(wi, wi) == 0)
		{
			pdf = 0;
		}
		else
		{
			wi = normalize(wi);
			// Convert from area measure, as returned by the Sample() call
			// above, to solid angle measure.
			pdf *= distanceSquared(ref.p, intr.p) / absDot(intr.n, -wi);
			if (std::isinf(pdf))
				pdf = 0.f;
		}
		return intr;
	}

	Float Shape::pdf(const Interaction &ref, const Vec3f &wi) const
	{
		// Intersect sample ray with area light geometry
		Ray ray = ref.spawnRay(wi);
		Float tHit;
		SurfaceInteraction isectLight;
		// Ignore any alpha textures used for trimming the shape when performing
		// this intersection. Hack for the "San Miguel" scene, where this is used
		// to make an invisible area light.
		if (!hit(ray, tHit, isectLight))
			return 0;

		// Convert light sample weight to solid angle measure
		Float pdf = distanceSquared(ref.p, isectLight.p) / (absDot(isectLight.n, -wi) * area());
		if (std::isinf(pdf))
			pdf = 0.f;
		return pdf;
	}

	Float Shape::solidAngle(const Vec3f &p, int nSamples) const
	{
		//AInteraction ref(p, Vec3f(), Vec3f(), Vec3f(0, 0, 1), 0);
		//double solidAngle = 0;
		//for (int i = 0; i < nSamples; ++i) 
		//{
		//	Vec2f u{ radicalInverse(0, i), radicalInverse(1, i) };
		//	Float pdf;
		//	AInteraction pShape = sample(ref, u, pdf);
		//	if (pdf > 0 && !hit(ARay(p, pShape.p - p, .999f))) 
		//	{
		//		solidAngle += 1 / pdf;
		//	}
		//}
		//return solidAngle / nSamples;
		return 1.0f;
	}

}