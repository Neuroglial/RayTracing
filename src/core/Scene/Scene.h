#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Render/Light.h"
#include "Object/Entity.h"
#include "Object/Object.h"

#include <vector>

namespace RT
{
	class Scene final
	{
	public:
		typedef std::shared_ptr<Scene> ptr;

		Scene(const std::vector<Entity::ptr> &entities, const HitableAggregate::ptr &aggre,
			const std::vector<Light::ptr> &lights)
			: m_lights(lights), m_aggreShape(aggre), m_entities(entities)
		{
			m_worldBound = m_aggreShape->worldBound();
			for (const auto &light : lights)
			{
				light->preprocess(*this);
				if (light->m_flags & (int)LightFlags::LightInfinite)
					m_infiniteLights.push_back(light);
			}
		}

		const BBox3f &worldBound() const { return m_worldBound; }

		bool hit(const Ray &ray) const;
		bool hit(const Ray &ray, SurfaceInteraction &isect) const;
		bool hitTr(Ray ray, Sampler &sampler, SurfaceInteraction &isect, Spectrum &transmittance) const;

		std::vector<Light::ptr> m_lights;
		// 
		std::vector<Light::ptr> m_infiniteLights;

	private:
		// Scene Private Data
		BBox3f m_worldBound;
		HitableAggregate::ptr m_aggreShape;
		std::vector<Entity::ptr> m_entities;
	};
}