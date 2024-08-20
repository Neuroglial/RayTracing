#include "Render/Render.h"

#include "Render/BSDF.h"
#include "Scene/Scene.h"
#include "Utils/Memory.h"
#include "Render/RenderReporter.h"
#include "Utils/LightDistrib.h"
#include "Utils/Parallel.h"

namespace RT
{
	//-------------------------------------------SamplerRenderer-------------------------------------

	void SamplerRenderer::render(const Scene &scene)
	{
		Vec2i resolution = m_camera->m_film->getResolution();

		auto &sampler = m_sampler;

		// 计算总的瓦片数量进行并行渲染
		BBox2i sampleBounds = m_camera->m_film->getSampleBounds();
		Vec2i sampleExtent = sampleBounds.diagonal();
		//一个瓦片为16*16像素
		constexpr int tileSize = 16;
		Vec2i nTiles((sampleExtent.x + tileSize - 1) / tileSize, (sampleExtent.y + tileSize - 1) / tileSize);

		AReporter reporter(nTiles.x * nTiles.y, "Rendering");

	 	ParallelUtils::parallelFor((size_t)0, (size_t)(nTiles.x * nTiles.y), [&](const size_t &t)
		{
			
			Vec2i tile(t % nTiles.x, t / nTiles.x);
			MemoryArena arena;

			//为每个线程新建一个采样器
			int seed = t;
			std::unique_ptr<Sampler> tileSampler = sampler->clone(seed);

			// 计算瓦片边界
			int x0 = sampleBounds.m_pMin.x + tile.x * tileSize;
			int x1 = glm::min(x0 + tileSize, sampleBounds.m_pMax.x);
			int y0 = sampleBounds.m_pMin.y + tile.y * tileSize;
			int y1 = glm::min(y0 + tileSize, sampleBounds.m_pMax.y);
			BBox2i tileBounds(Vec2i(x0, y0), Vec2i(x1, y1));

			// 获取渲染瓦片
			std::unique_ptr<FilmTile> filmTile = m_camera->m_film->getFilmTile(tileBounds);

			// 逐像素遍历
			for (Vec2i pixel : tileBounds)
			{
				//开始采样
				tileSampler->startPixel(pixel);

				do
				{
					// 为当前样本初始化CameraSample
					CameraSample cameraSample = tileSampler->getCameraSample(pixel);

					// 生成当前点的射线，并计算权值
					Ray ray;
					Float rayWeight = m_camera->castingRay(cameraSample, ray);

					// 进行光线最总
					Spectrum L(0.f);
					if (rayWeight > 0)
					{
						L = Li(ray, scene, *tileSampler, arena);
					}

					// 处理异常返回
					if (L.hasNaNs())
					{
						L = Spectrum(0.f);
					}
					else if (L.luminance() < -1e-5)
					{
						L = Spectrum(0.f);
					}
					else if (std::isinf(L.luminance()))
					{
						L = Spectrum(0.f);
					}

					// 将当前采样添加到film中
					filmTile->addSample(cameraSample.pFilm, L, rayWeight);

					// 从计算图像样本值中释放MemoryRena内存
					arena.Reset();

					//小于最大采样次数继续采样
				} while (tileSampler->startNextSample());
			}

			m_camera->m_film->mergeFilmTile(std::move(filmTile));
			reporter.update();
			
		}, ExecutionPolicy::APARALLEL);

		reporter.done();

		m_camera->m_film->writeImageToFile();

	}

	Spectrum SamplerRenderer::specularReflect(const Ray &ray, const SurfaceInteraction &isect,
		const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const
	{
		// Compute specular reflection direction _wi_ and BSDF value
		Vec3f wo = isect.wo, wi;
		Float pdf;
		ABxDFType sampledType;
		ABxDFType type = ABxDFType(BSDF_REFLECTION | BSDF_SPECULAR);

		Spectrum f = isect.bsdf->sample_f(wo, wi, sampler.get2D(), pdf, sampledType, type);

		// Return contribution of specular reflection
		const Vec3f &ns = isect.n;

		if (pdf > 0.f && !f.isBlack() && absDot(wi, ns) != 0.f)
		{
			// Compute ray differential _rd_ for specular reflection
			Ray rd = isect.spawnRay(wi);
			return f * Li(rd, scene, sampler, arena, depth + 1) * absDot(wi, ns) / pdf;
		}
		else
		{
			return Spectrum(0.f);
		}
	}

	Spectrum SamplerRenderer::specularTransmit(const Ray &ray, const SurfaceInteraction &isect,
		const Scene &scene, Sampler &sampler, MemoryArena &arena, int depth) const
	{
		Vec3f wo = isect.wo, wi;
		Float pdf;
		const Vec3f &p = isect.p;
		const BSDF &bsdf = *(isect.bsdf);
		ABxDFType type = ABxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR);
		ABxDFType sampledType;
		Spectrum f = bsdf.sample_f(wo, wi, sampler.get2D(), pdf, sampledType, type);
		Spectrum L = Spectrum(0.f);
		Vec3f ns = isect.n;

		if (pdf > 0.f && !f.isBlack() && absDot(wi, ns) != 0.f)
		{
			// Compute ray differential _rd_ for specular transmission
			Ray rd = isect.spawnRay(wi);
			L = f * Li(rd, scene, sampler, arena, depth + 1) * absDot(wi, ns) / pdf;
		}
		return L;
	}

	//------------------------------------------Utility functions-------------------------------------

	Spectrum uiformSampleAllLights(const Interaction &it, const Scene &scene,
		MemoryArena &arena, Sampler &sampler, const std::vector<int> &nLightSamples)
	{
		Spectrum L(0.f);
		for (size_t j = 0; j < scene.m_lights.size(); ++j)
		{
			// Accumulate contribution of _j_th light to _L_
			const Light::ptr &light = scene.m_lights[j];
			int nSamples = nLightSamples[j];
			const Vec2f *uLightArray = sampler.get2DArray(nSamples);
			const Vec2f *uScatteringArray = sampler.get2DArray(nSamples);

			if (!uLightArray || !uScatteringArray)
			{
				// Use a single sample for illumination from _light_
				Vec2f uLight = sampler.get2D();
				Vec2f uScattering = sampler.get2D();
				L += estimateDirect(it, uScattering, *light, uLight, scene, sampler, arena);
			}
			else
			{
				// Estimate direct lighting using sample arrays
				Spectrum Ld(0.f);
				for (int k = 0; k < nSamples; ++k)
				{
					Ld += estimateDirect(it, uScatteringArray[k], *light, uLightArray[k], scene, sampler, arena);
				}
				L += Ld / nSamples;
			}
		}
		return L;
	}

	Spectrum uniformSampleOneLight(const Interaction &it, const Scene &scene,
		MemoryArena &arena, Sampler &sampler, const Distribution1D *lightDistrib)
	{
		// 随机选择单个灯光进行采样
		int nLights = int(scene.m_lights.size());

		if (nLights == 0)
			return Spectrum(0.f);

		int lightSampledIndex;
		Float lightPdf;

		if (lightDistrib != nullptr) 
		{
			lightSampledIndex = lightDistrib->sampleDiscrete(sampler.get1D(), &lightPdf);
			if (lightPdf == 0) 
				return Spectrum(0.f);
		}
		else 
		{
			lightSampledIndex = glm::min((int)(sampler.get1D() * nLights), nLights - 1);
			lightPdf = Float(1) / nLights;
		}

		const Light::ptr &light = scene.m_lights[lightSampledIndex];
		Vec2f uLight = sampler.get2D();
		Vec2f uScattering = sampler.get2D();

		return estimateDirect(it, uScattering, *light, uLight, scene, sampler, arena) / lightPdf;
	}

	Spectrum estimateDirect(const Interaction &it, const Vec2f &uScattering, const Light &light,
		const Vec2f &uLight, const Scene &scene, Sampler &sampler, MemoryArena &arena, bool specular)
	{
		ABxDFType bsdfFlags = specular ? BSDF_ALL : ABxDFType(BSDF_ALL & ~BSDF_SPECULAR);

		Spectrum Ld(0.f);
		// Sample light source with multiple importance sampling
		Vec3f wi;
		Float lightPdf = 0, scatteringPdf = 0;
		VisibilityTester visibility;
		Spectrum Li = light.sample_Li(it, uLight, wi, lightPdf, visibility);

		if (lightPdf > 0 && !Li.isBlack())
		{
			// Compute BSDF or phase function's value for light sample
			Spectrum f;
			// Evaluate BSDF for light sampling strategy
			const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
			f = isect.bsdf->f(isect.wo, wi, bsdfFlags) * absDot(wi, isect.n);

			scatteringPdf = isect.bsdf->pdf(isect.wo, wi, bsdfFlags);

			if (!f.isBlack())
			{
				// Compute effect of visibility for light source sample
				if (!visibility.unoccluded(scene))
				{
					Li = Spectrum(0.f);
				}

				// Add light's contribution to reflected radiance
				if (!Li.isBlack())
				{
					if (isDeltaLight(light.m_flags))
					{
						Ld += f * Li / lightPdf;
					}
					else
					{
						Float weight = powerHeuristic(1, lightPdf, 1, scatteringPdf);
						Ld += f * Li * weight / lightPdf;
					}
				}
			}
		}

		// Sample BSDF with multiple importance sampling
		if (!isDeltaLight(light.m_flags))
		{
			Spectrum f;
			bool sampledSpecular = false;
			// Sample scattered direction for surface interactions
			ABxDFType sampledType;
			const SurfaceInteraction &isect = (const SurfaceInteraction &)it;
			f = isect.bsdf->sample_f(isect.wo, wi, uScattering, scatteringPdf, sampledType, bsdfFlags);
			f *= absDot(wi, isect.n);
			sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;

			if (!f.isBlack() && scatteringPdf > 0)
			{
				// Account for light contributions along sampled direction _wi_
				Float weight = 1;
				if (!sampledSpecular)
				{
					lightPdf = light.pdf_Li(it, wi);
					if (lightPdf == 0) 
						return Ld;
					weight = powerHeuristic(1, scatteringPdf, 1, lightPdf);
				}

				// Find intersection and compute transmittance
				SurfaceInteraction lightIsect;
				Ray ray = it.spawnRay(wi);
				Spectrum Tr(1.f);
				bool foundSurfaceInteraction = scene.hit(ray, lightIsect);

				// Add light contribution from material sampling
				Spectrum Li(0.f);
				if (foundSurfaceInteraction)
				{
					if (lightIsect.hitable->getAreaLight() == &light)
						Li = lightIsect.Le(-wi);
				}
				else
				{
					Li = light.Le(ray);
				}
				if (!Li.isBlack())
					Ld += f * Li * Tr * weight / scatteringPdf;
			}
		}
		return Ld;
	}

}

namespace RT
{
	AURORA_REGISTER_CLASS(PathRenderer, "Path")

		PathRenderer::PathRenderer(const PropertyTreeNode& node)
		: SamplerRenderer(nullptr, nullptr), m_maxDepth(node.getPropertyList().getInteger("Depth", 2))
		, m_rrThreshold(1.f), m_lightSampleStrategy("spatial")
	{
		//Sampler
		const auto& samplerNode = node.getPropertyChild("Sampler");
		m_sampler = Sampler::ptr(static_cast<Sampler*>(ObjectFactory::createInstance(
			samplerNode.getTypeName(), samplerNode)));

		//Camera
		const auto& cameraNode = node.getPropertyChild("Camera");
		m_camera = Camera::ptr(static_cast<Camera*>(ObjectFactory::createInstance(
			cameraNode.getTypeName(), cameraNode)));

		activate();
	}

	PathRenderer::PathRenderer(int maxDepth, Camera::ptr camera, Sampler::ptr sampler,
		Float rrThreshold, const std::string& lightSampleStrategy)
		: SamplerRenderer(camera, sampler), m_maxDepth(maxDepth),
		m_rrThreshold(rrThreshold), m_lightSampleStrategy(lightSampleStrategy) {}

	void PathRenderer::preprocess(const Scene& scene)
	{
		m_lightDistribution = createLightSampleDistribution(m_lightSampleStrategy, scene);
	}


	Spectrum PathRenderer::Li(const Ray& r, const Scene& scene, Sampler& sampler,
		MemoryArena& arena, int depth) const
	{
		//初始化
		Spectrum L(0.f), beta(1.f);
		Ray ray(r);

		//是否为镜面反射
		bool specularBounce = false;
		//反射次数
		int bounces;

		//折射比例
		Float etaScale = 1;

		for (bounces = 0;; ++bounces)
		{
			// 查找下一个路径顶点并累积贡献

			// 将ray与场景相交，并将相交点存储在isect中
			SurfaceInteraction isect;
			bool hit = scene.hit(ray, isect);

			if (bounces == 0 || specularBounce)
			{
				// 在路径顶点或环境中添加发射光
				if (hit)
				{
					L += beta * isect.Le(-ray.direction());
				}
				else
				{
					for (const auto& light : scene.m_infiniteLights)
						L += beta * light->Le(ray);
				}
			}

			// 如果光线逃逸或达到maxDepth，则终止路径
			if (!hit || bounces >= m_maxDepth)
				break;

			// 计算散射函数
			isect.computeScatteringFunctions(ray, arena, true);

			// bsdf==nullptr表示当前表面对光没有影响，这样的表面用于表示参与介质之间的过渡，其边界本身是光学非活动的。
			if (!isect.bsdf){
				ray = isect.spawnRay(ray.direction());
				bounces--;
				continue;
			}

			const Distribution1D* distrib = m_lightDistribution->lookup(isect.p);

			// 从灯光中采样照明以查找路径贡献。（但对于完全镜面反射的BSDF，跳过此步骤）
			if (isect.bsdf->numComponents(ABxDFType(BSDF_ALL & ~BSDF_SPECULAR)) > 0)
			{
				Spectrum Ld = beta * uniformSampleOneLight(isect, scene, arena, sampler, distrib);
				CHECK_GE(Ld.luminance(), 0.f);
				L += Ld;
			}

			// Sample BSDF to get new path direction
			Vec3f wo = -ray.direction(), wi;
			Float pdf;
			ABxDFType flags;
			Spectrum f = isect.bsdf->sample_f(wo, wi, sampler.get2D(), pdf, flags, BSDF_ALL);

			if (f.isBlack() || pdf == 0.f)
				break;
			beta *= f * absDot(wi, isect.n) / pdf;

			CHECK_GE(beta.luminance(), 0.f);
			DCHECK(!glm::isinf(beta.luminance()));

			specularBounce = (flags & BSDF_SPECULAR) != 0;
			if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION))
			{
				Float eta = isect.bsdf->m_eta;
				// Update the term that tracks radiance scaling for refraction
				// depending on whether the ray is entering or leaving the
				// medium.
				etaScale *= (dot(wo, isect.n) > 0) ? (eta * eta) : 1 / (eta * eta);
			}

			ray = isect.spawnRay(wi);

			// Possibly terminate the path with Russian roulette.
			// Factor out radiance scaling due to refraction in rrBeta.
			Spectrum rrBeta = beta * etaScale;
			if (rrBeta.maxComponentValue() < m_rrThreshold && bounces > 3)
			{
				Float q = glm::max((Float).05f, 1 - rrBeta.maxComponentValue());
				if (sampler.get1D() < q)
					break;
				beta /= 1 - q;
				DCHECK(!glm::isinf(beta.luminance()));
			}
		}

		//ReportValue(pathLength, bounces);
		return L;
	}

}

namespace RT
{
	AURORA_REGISTER_CLASS(WhittedRenderer, "Whitted")

		WhittedRenderer::WhittedRenderer(const PropertyTreeNode& node)
		:SamplerRenderer(nullptr, nullptr), m_maxDepth(node.getPropertyList().getInteger("Depth", 2))
	{
		//Sampler
		const auto& samplerNode = node.getPropertyChild("Sampler");
		m_sampler = Sampler::ptr(static_cast<Sampler*>(ObjectFactory::createInstance(
			samplerNode.getTypeName(), samplerNode)));

		//Camera
		const auto& cameraNode = node.getPropertyChild("Camera");
		m_camera = Camera::ptr(static_cast<Camera*>(ObjectFactory::createInstance(
			cameraNode.getTypeName(), cameraNode)));

		activate();
	}

	Spectrum WhittedRenderer::Li(const Ray& ray, const Scene& scene,
		Sampler& sampler, MemoryArena& arena, int depth) const
	{
		Spectrum L(0.);

		SurfaceInteraction isect;

		// No intersection found, just return lights emission
		if (!scene.hit(ray, isect))
		{
			for (const auto& light : scene.m_lights)
				L += light->Le(ray);
			return L;
		}

		// Compute emitted and reflected light at ray intersection point

		// Initialize common variables for Whitted Renderer
		const Vec3f& n = isect.n;
		Vec3f wo = isect.wo;

		// Calculate BSDF function for surface interaction
		isect.computeScatteringFunctions(ray, arena);

		// There is no bsdf funcion
		if (!isect.bsdf)
		{
			return Li(isect.spawnRay(ray.direction()), scene, sampler, arena, depth);
		}

		// Compute emitted light if ray hit an area light source -> Le (emission term)
		L += isect.Le(wo);

		// Add contribution of each light source -> shadow ray
		for (const auto& light : scene.m_lights)
		{
			Vec3f wi;
			Float pdf;
			VisibilityTester visibility;
			Spectrum Li = light->sample_Li(isect, sampler.get2D(), wi, pdf, visibility);

			if (Li.isBlack() || pdf == 0)
				continue;

			Spectrum f = isect.bsdf->f(wo, wi);
			if (!f.isBlack() && visibility.unoccluded(scene))
			{
				L += f * Li * absDot(wi, n) / pdf;
			}
		}

		if (depth + 1 < m_maxDepth)
		{
			// Trace rays for specular reflection and refraction
			L += specularReflect(ray, isect, scene, sampler, arena, depth);
			L += specularTransmit(ray, isect, scene, sampler, arena, depth);
		}

		return L;
	}
}