#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Render/Sampler.h"
#include "Camera/Camera.h"
#include "Object/Hitable.h"
#include "Object/Object.h"
#include "Utils/LightDistrib.h"

namespace RT
{
	class Renderer : public Object
	{
	public:
		typedef std::shared_ptr<Renderer> ptr;

		virtual ~Renderer() = default;

		virtual void preprocess(const Scene &scene) = 0;
		virtual void render(const Scene &scene) = 0;

		virtual AClassType getClassType() const override { return AClassType::AERenderer; }

	};

	class SamplerRenderer : public Renderer
	{
	public:
		typedef std::shared_ptr<SamplerRenderer> ptr;

		SamplerRenderer(Camera::ptr camera, Sampler::ptr sampler)
			: m_camera(camera), m_sampler(sampler) {}

		virtual void preprocess(const Scene &scene) override {}

		virtual void render(const Scene &scene) override;

		virtual Spectrum Li(const Ray &ray, const Scene &scene,
			Sampler &sampler, MemoryArena &arena, int depth = 0) const = 0;

		Spectrum specularReflect(const Ray &ray, const SurfaceInteraction &isect,
			const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const;

		Spectrum specularTransmit(const Ray &ray, const SurfaceInteraction &isect,
			const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const;

	protected:
		Camera::ptr m_camera;
		Sampler::ptr m_sampler;
	};

	Spectrum uiformSampleAllLights(const Interaction &it, const Scene &scene,
		MemoryArena &arena, Sampler &sampler, const std::vector<int> &nLightSamples);

	Spectrum uniformSampleOneLight(const Interaction &it, const Scene &scene,
		MemoryArena &arena, Sampler &sampler, const Distribution1D *lightDistrib);

	Spectrum estimateDirect(const Interaction &it, const Vec2f &uShading, const Light &light,
		const Vec2f &uLight, const Scene &scene, Sampler &sampler, MemoryArena &arena, bool specular = false);

}

namespace RT
{
	class PathRenderer : public SamplerRenderer
	{
	public:

		PathRenderer(const PropertyTreeNode& props);

		PathRenderer(int maxDepth, Camera::ptr camera, Sampler::ptr sampler,
			Float rrThreshold = 1, const std::string& lightSampleStrategy = "spatial");

		virtual void preprocess(const Scene& scene) override;

		virtual Spectrum Li(const Ray& ray, const Scene& scene, Sampler& sampler,
			MemoryArena& arena, int depth) const override;

		virtual std::string toString() const override { return "PathRenderer[]"; }

	private:
		// PathRenderer Private Data
		int m_maxDepth;
		Float m_rrThreshold;
		std::string m_lightSampleStrategy;
		std::unique_ptr<ALightDistribution> m_lightDistribution;
	};

}

namespace RT
{
	class WhittedRenderer : public SamplerRenderer
	{
	public:
		typedef std::shared_ptr<WhittedRenderer> ptr;

		WhittedRenderer(const PropertyTreeNode& node);
		WhittedRenderer(int maxDepth, Camera::ptr camera, Sampler::ptr sampler)
			: SamplerRenderer(camera, sampler), m_maxDepth(maxDepth) {}

		virtual Spectrum Li(const Ray& ray, const Scene& scene,
			Sampler& sampler, MemoryArena& arena, int depth) const override;

		virtual std::string toString() const override { return "WhittedRenderer[]"; }

	private:
		const int m_maxDepth;
	};
}