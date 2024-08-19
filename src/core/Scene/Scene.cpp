#include "Scene/Scene.h"

namespace RT
{
	bool Scene::hit(const Ray &ray, SurfaceInteraction &isect) const
	{
		//DCHECK_NE(ray.direction(), Vec3f(0, 0, 0));
		return m_aggreShape->hit(ray, isect);
	}

	bool Scene::hit(const Ray &ray) const
	{
		//DCHECK_NE(ray.direction(), Vec3f(0, 0, 0));
		return m_aggreShape->hit(ray);
	}

	bool Scene::hitTr(Ray ray, Sampler &sampler, SurfaceInteraction &isect, Spectrum &Tr) const
	{
		Tr = Spectrum(1.f);
		//while (true) {
		//	bool hitSurface = hit(ray, isect);
		//	// Accumulate beam transmittance for ray segment
		//	if (ray.medium) *Tr *= ray.medium->Tr(ray, sampler);

		//	// Initialize next ray segment or terminate transmittance computation
		//	if (!hitSurface) return false;
		//	if (isect->primitive->GetMaterial() != nullptr) return true;
		//	ray = isect->SpawnRay(ray.d);
		//}
		return false;
	}
}