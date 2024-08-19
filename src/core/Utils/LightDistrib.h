#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"

#include <vector>

namespace RT
{
	class Distribution1D 
	{
	public:

		Distribution1D(const Float *f, int n) : func(f, f + n), cdf(n + 1) 
		{
			// Compute integral of step function at $x_i$
			cdf[0] = 0;
			for (int i = 1; i < n + 1; ++i) 
				cdf[i] = cdf[i - 1] + func[i - 1] / n;

			// Transform step function integral into CDF
			funcInt = cdf[n];
			if (funcInt == 0) 
			{
				for (int i = 1; i < n + 1; ++i) 
					cdf[i] = Float(i) / Float(n);
			}
			else 
			{
				for (int i = 1; i < n + 1; ++i) 
					cdf[i] /= funcInt;
			}
		}

		int count() const 
		{ 
			return (int)func.size();
		}

		Float sampleContinuous(Float u, Float *pdf, int *off = nullptr) const 
		{
			// Find surrounding CDF segments and _offset_
			int offset = findInterval((int)cdf.size(), [&](int index) 
			{ 
				return cdf[index] <= u; 
			});

			if (off) 
				*off = offset;

			// Compute offset along CDF segment
			Float du = u - cdf[offset];
			if ((cdf[offset + 1] - cdf[offset]) > 0) 
			{
				CHECK_GT(cdf[offset + 1], cdf[offset]);
				du /= (cdf[offset + 1] - cdf[offset]);
			}
			DCHECK(!glm::isnan(du));

			// Compute PDF for sampled offset
			if (pdf) 
				*pdf = (funcInt > 0) ? func[offset] / funcInt : 0;

			// Return $x\in{}[0,1)$ corresponding to sample
			return (offset + du) / count();
		}

		int sampleDiscrete(Float u, Float *pdf = nullptr, Float *uRemapped = nullptr) const 
		{
			// Find surrounding CDF segments and _offset_
			int offset = findInterval((int)cdf.size(), [&](int index) 
			{ 
				return cdf[index] <= u; 
			});

			if (pdf) 
				*pdf = (funcInt > 0) ? func[offset] / (funcInt * count()) : 0;
			if (uRemapped)
				*uRemapped = (u - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
			if (uRemapped) 
				CHECK(*uRemapped >= 0.f && *uRemapped <= 1.f);
			return offset;
		}

		Float discretePDF(int index) const 
		{
			CHECK(index >= 0 && index < count());
			return func[index] / (funcInt * count());
		}

		std::vector<Float> func, cdf;
		Float funcInt;
	};

	// LightDistribution为类定义了一个通用接口，这些类为在空间中的给定点对光源进行采样提供概率分布
	class ALightDistribution 
	{
	public:
		virtual ~ALightDistribution() = default;

		// Given a point |p| in space, this method returns a (hopefully
		// effective) sampling distribution for light sources at that point.
		virtual const Distribution1D *lookup(const Vec3f &p) const = 0;
	};

	// LightDistribution最简单的可能实现：忽略提供的点，在所有光源上返回均匀分布。这种方法适用于非常简单的场景，但对于具有多个光源的场景则效果不佳。
	class AUniformLightDistribution : public ALightDistribution 
	{
	public:
		
		AUniformLightDistribution(const Scene &scene);

		virtual const Distribution1D *lookup(const Vec3f &p) const override;

	private:
		std::unique_ptr<Distribution1D> distrib;
	};

	std::unique_ptr<ALightDistribution> createLightSampleDistribution(
		const std::string &name, const Scene &scene);

}