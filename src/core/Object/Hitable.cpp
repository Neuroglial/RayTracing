#include "Object/Hitable.h"

#include "Utils/Interaction.h"

namespace RT
{
	//-------------------------------------------AHitableObject-------------------------------------

	HitableObject::HitableObject(const Shape::ptr &shape, const Material* material,
		const AreaLight::ptr &areaLight)
		: m_shape(shape), m_material(material), m_areaLight(areaLight) 
	{
		if (m_areaLight != nullptr)
		{
			m_areaLight->setParent(this);
		}
	}

	bool HitableObject::hit(const Ray &ray) const { return m_shape->hit(ray); }

	bool HitableObject::hit(const Ray &ray, SurfaceInteraction &isect) const
	{
		Float tHit;
		if (!m_shape->hit(ray, tHit, isect))
			return false;

		ray.m_tMax = tHit;
		isect.hitable = this;
		return true;
	}

	void HitableObject::computeScatteringFunctions(SurfaceInteraction &isect, MemoryArena &arena,
		ATransportMode mode, bool allowMultipleLobes) const
	{
		if (m_material != nullptr)
		{
			m_material->computeScatteringFunctions(isect, arena, mode, allowMultipleLobes);
		}
	}

	BBox3f HitableObject::worldBound() const { return m_shape->worldBound(); }

	Shape* HitableObject::getShape() const { return m_shape.get(); }

	const AreaLight* HitableObject::getAreaLight() const { return m_areaLight.get(); }

	const Material* HitableObject::getMaterial() const { return m_material; }

	//-------------------------------------------AHitableAggregate-------------------------------------

	const AreaLight *HitableAggregate::getAreaLight() const { return nullptr; }

	const Material *HitableAggregate::getMaterial() const { return nullptr; }

	void HitableAggregate::computeScatteringFunctions(SurfaceInteraction &isect, MemoryArena &arena,
		ATransportMode mode, bool allowMultipleLobes) const
	{
		//Note: should not go here at all.
		LOG(FATAL) <<
			"AHitableAggregate::computeScatteringFunctions() shouldn't be "
			"called";
	}

}