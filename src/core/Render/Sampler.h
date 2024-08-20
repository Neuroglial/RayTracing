#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Object/Object.h"

#include <vector>

namespace RT
{
	class Sampler : public Object
	{
	public:
		typedef std::shared_ptr<Sampler> ptr;

		virtual ~Sampler();
		Sampler(const PropertyList &props);
		Sampler(int64_t samplesPerPixel);

		virtual void startPixel(const Vec2i &p);
		virtual Float get1D() = 0;
		virtual Vec2f get2D() = 0;
		CameraSample getCameraSample(const Vec2i &pRaster);

		void request1DArray(int n);
		void request2DArray(int n);

		virtual int roundCount(int n) const { return n; }

		const Float *get1DArray(int n);
		const Vec2f *get2DArray(int n)
			;
		virtual bool startNextSample();

		virtual std::unique_ptr<Sampler> clone(int seed) = 0;
		virtual bool setSampleNumber(int64_t sampleNum);

		int64_t currentSampleNumber() const { return m_currentPixelSampleIndex; }

		int64_t getSamplingNumber() const { return samplesPerPixel; }

		virtual ClassType getClassType() const override { return ClassType::AESampler; }

		const int64_t samplesPerPixel; //Number of sampling per pixel

	protected:
		Vec2i m_currentPixel;
		int64_t m_currentPixelSampleIndex;
		std::vector<int> m_samples1DArraySizes, m_samples2DArraySizes;
		std::vector<std::vector<Float>> m_sampleArray1D;
		std::vector<std::vector<Vec2f>> m_sampleArray2D;

	private:
		size_t m_array1DOffset, m_array2DOffset;
	};

	Vec3f uniformSampleHemisphere(const Vec2f &u);
	Float uniformHemispherePdf();
	Vec3f uniformSampleSphere(const Vec2f &u);
	Float uniformSpherePdf();

	Vec3f uniformSampleCone(const Vec2f &u, Float thetamax);
	Vec3f uniformSampleCone(const Vec2f &u, Float thetamax, const Vec3f &x,
		const Vec3f &y, const Vec3f &z);
	Float uniformConePdf(Float thetamax);

	Vec2f concentricSampleDisk(const Vec2f &u);

	Vec2f uniformSampleTriangle(const Vec2f &u);

	inline Vec3f cosineSampleHemisphere(const Vec2f &u)
	{
		Vec2f d = concentricSampleDisk(u);
		Float z = std::sqrt(glm::max((Float)0, 1 - d.x * d.x - d.y * d.y));
		return Vec3f(d.x, d.y, z);
	}

	inline Float cosineHemispherePdf(Float cosTheta) { return cosTheta * InvPi; }

	inline Float balanceHeuristic(int nf, Float fPdf, int ng, Float gPdf)
	{
		return (nf * fPdf) / (nf * fPdf + ng * gPdf);
	}

	inline Float powerHeuristic(int nf, Float fPdf, int ng, Float gPdf)
	{
		Float f = nf * fPdf, g = ng * gPdf;
		return (f * f) / (f * f + g * g);
	}
}

namespace RT
{
	class ARandomSampler final : public Sampler
	{
	public:
		typedef std::shared_ptr<ARandomSampler> ptr;

		ARandomSampler(const PropertyTreeNode& node);
		ARandomSampler(int ns, int seed = 0);

		virtual void startPixel(const Vec2i&) override;
		
		//Ëæ»úÆ«ÒÆ
		virtual Float get1D() override;
		virtual Vec2f get2D() override;

		virtual std::unique_ptr<Sampler> clone(int seed) override;

		virtual std::string toString() const override { return "RandomSampler[]"; }

	private:
		Rng m_rng; //Random number generator
	};
}