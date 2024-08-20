#include "Utils/LightDistrib.h"

#include "Scene/Scene.h"

namespace RT
{
	std::unique_ptr<LightDistribution> createLightSampleDistribution(
		const std::string &name, const Scene &scene) 
	{
		{
			return std::unique_ptr<LightDistribution>{
				new UniformLightDistribution(scene)};
		}

	}

	UniformLightDistribution::UniformLightDistribution(const Scene &scene) 
	{
		std::vector<Float> prob(scene.m_lights.size(), Float(1));
		distrib.reset(new Distribution1D(&prob[0], int(prob.size())));
	}

	const Distribution1D *UniformLightDistribution::lookup(const Vec3f &p) const 
	{
		return distrib.get();
	}
}