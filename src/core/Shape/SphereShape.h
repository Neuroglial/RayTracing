#ifndef ARSPHERERSHAPE_H
#define ARSPHERERSHAPE_H

#include "Shape/Shape.h"

namespace RT
{
	class ASphereShape final : public Shape
	{
	public:
		typedef std::shared_ptr<ASphereShape> ptr;

		ASphereShape(const PropertyTreeNode &node);
		ASphereShape(Transform *objectToWorld, Transform *worldToObject, const float radius);

		virtual ~ASphereShape() = default;

		virtual Float area() const override;

		virtual Interaction sample(const Vec2f &u, Float &pdf) const override;

		virtual Interaction sample(const Interaction &ref, const Vec2f &u, Float &pdf) const override;
		virtual Float pdf(const Interaction &ref, const Vec3f &wi) const override;

		virtual BBox3f objectBound() const override;

		virtual bool hit(const Ray &ray) const override;
		virtual bool hit(const Ray &ray, Float &tHit, SurfaceInteraction &isect) const override;

		virtual Float solidAngle(const Vec3f &p, int nSamples = 512) const override;

		virtual std::string toString() const override { return "SphereShape[]"; }

	private:
		float m_radius;
	};
}

#endif