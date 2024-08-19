#include "Render/Sampler.h"

#include "Camera/Camera.h"

namespace RT
{
	//-------------------------------------------ASampler-------------------------------------

	Sampler::~Sampler() {}

	Sampler::Sampler(int64_t samplesPerPixel) : samplesPerPixel(samplesPerPixel) {}

	Sampler::Sampler(const PropertyList &props) : samplesPerPixel(props.getInteger("SPP", 1)) {}

	CameraSample Sampler::getCameraSample(const Vec2i &pRaster)
	{
		CameraSample cs;
		cs.pFilm = (Vec2f)pRaster + get2D();
		return cs;
	}

	void Sampler::startPixel(const Vec2i &p)
	{
		m_currentPixel = p;
		m_currentPixelSampleIndex = 0;
		// Reset array offsets for next pixel sample
		m_array1DOffset = m_array2DOffset = 0;
	}

	bool Sampler::startNextSample()
	{
		// Reset array offsets for next pixel sample
		m_array1DOffset = m_array2DOffset = 0;
		return ++m_currentPixelSampleIndex < samplesPerPixel;
	}

	bool Sampler::setSampleNumber(int64_t sampleNum)
	{
		// Reset array offsets for next pixel sample
		m_array1DOffset = m_array2DOffset = 0;
		m_currentPixelSampleIndex = sampleNum;
		return m_currentPixelSampleIndex < samplesPerPixel;
	}

	void Sampler::request1DArray(int n)
	{
		CHECK_EQ(roundCount(n), n);
		m_samples1DArraySizes.push_back(n);
		m_sampleArray1D.push_back(std::vector<Float>(n * samplesPerPixel));
	}

	void Sampler::request2DArray(int n)
	{
		CHECK_EQ(roundCount(n), n);
		m_samples2DArraySizes.push_back(n);
		m_sampleArray2D.push_back(std::vector<Vec2f>(n * samplesPerPixel));
	}

	const Float *Sampler::get1DArray(int n)
	{
		if (m_array1DOffset == m_sampleArray1D.size())
			return nullptr;
		CHECK_EQ(m_samples1DArraySizes[m_array1DOffset], n);
		CHECK_LT(m_currentPixelSampleIndex, samplesPerPixel);
		return &m_sampleArray1D[m_array1DOffset++][m_currentPixelSampleIndex * n];
	}

	const Vec2f *Sampler::get2DArray(int n)
	{
		if (m_array2DOffset == m_sampleArray2D.size())
			return nullptr;
		CHECK_EQ(m_samples2DArraySizes[m_array2DOffset], n);
		CHECK_LT(m_currentPixelSampleIndex, samplesPerPixel);
		return &m_sampleArray2D[m_array2DOffset++][m_currentPixelSampleIndex * n];
	}

	//-------------------------------------------SamplingAlgorithm-------------------------------------

	Vec3f uniformSampleHemisphere(const Vec2f &u)
	{
		Float z = u[0];
		Float r = glm::sqrt(glm::max((Float)0, (Float)1. - z * z));
		Float phi = 2 * Pi * u[1];
		return Vec3f(r * glm::cos(phi), r * glm::sin(phi), z);
	}

	Float uniformHemispherePdf() { return Inv2Pi; }

	Vec3f uniformSampleSphere(const Vec2f &u)
	{
		Float z = 1 - 2 * u[0];
		Float r = glm::sqrt(glm::max((Float)0, (Float)1 - z * z));
		Float phi = 2 * Pi * u[1];
		return Vec3f(r * glm::cos(phi), r * glm::sin(phi), z);
	}

	Float uniformSpherePdf() { return Inv4Pi; }

	Vec3f uniformSampleCone(const Vec2f &u, Float cosThetaMax)
	{
		Float cosTheta = ((Float)1 - u[0]) + u[0] * cosThetaMax;
		Float sinTheta = glm::sqrt((Float)1 - cosTheta * cosTheta);
		Float phi = u[1] * 2 * Pi;
		return Vec3f(glm::cos(phi) * sinTheta, glm::sin(phi) * sinTheta, cosTheta);
	}

	Vec3f uniformSampleCone(const Vec2f &u, Float cosThetaMax, const Vec3f &x,
		const Vec3f &y, const Vec3f &z)
	{
		Float cosTheta = lerp(u[0], cosThetaMax, 1.f);
		Float sinTheta = glm::sqrt((Float)1. - cosTheta * cosTheta);
		Float phi = u[1] * 2 * Pi;
		return glm::cos(phi) * sinTheta * x + glm::sin(phi) * sinTheta * y + cosTheta * z;
	}

	Float uniformConePdf(Float cosThetaMax) { return 1 / (2 * Pi * (1 - cosThetaMax)); }

	Vec2f concentricSampleDisk(const Vec2f &u)
	{
		// Map uniform random numbers to $[-1,1]^2$
		Vec2f uOffset = 2.f * u - Vec2f(1, 1);

		// Handle degeneracy at the origin
		if (uOffset.x == 0 && uOffset.y == 0)
			return Vec2f(0, 0);

		// Apply concentric mapping to point
		Float theta, r;
		if (glm::abs(uOffset.x) > glm::abs(uOffset.y))
		{
			r = uOffset.x;
			theta = PiOver4 * (uOffset.y / uOffset.x);
		}
		else
		{
			r = uOffset.y;
			theta = PiOver2 - PiOver4 * (uOffset.x / uOffset.y);
		}
		return r * Vec2f(glm::cos(theta), glm::sin(theta));
	}

	Vec2f uniformSampleTriangle(const Vec2f &u)
	{
		Float su0 = glm::sqrt(u[0]);
		return Vec2f(1 - su0, u[1] * su0);
	}

}

namespace RT
{
	AURORA_REGISTER_CLASS(ARandomSampler, "Random")

		ARandomSampler::ARandomSampler(const PropertyTreeNode& node) : Sampler(node.getPropertyList()), m_rng(0) { activate(); }

	ARandomSampler::ARandomSampler(int ns, int seed) : Sampler(ns), m_rng(seed) {}

	Float ARandomSampler::get1D()
	{
		CHECK_LT(m_currentPixelSampleIndex, samplesPerPixel);
		return m_rng.uniformFloat();
	}

	Vec2f ARandomSampler::get2D()
	{
		CHECK_LT(m_currentPixelSampleIndex, samplesPerPixel);
		return { m_rng.uniformFloat(), m_rng.uniformFloat() };
	}

	std::unique_ptr<Sampler> ARandomSampler::clone(int seed)
	{
		ARandomSampler* rs = new ARandomSampler(*this);
		rs->m_rng.setSequence(seed);
		return std::unique_ptr<Sampler>(rs);
	}

	void ARandomSampler::startPixel(const Vec2i& p)
	{
		for (size_t i = 0; i < m_sampleArray1D.size(); ++i)
			for (size_t j = 0; j < m_sampleArray1D[i].size(); ++j)
				m_sampleArray1D[i][j] = m_rng.uniformFloat();

		for (size_t i = 0; i < m_sampleArray2D.size(); ++i)
			for (size_t j = 0; j < m_sampleArray2D[i].size(); ++j)
				m_sampleArray2D[i][j] = { m_rng.uniformFloat(), m_rng.uniformFloat() };

		Sampler::startPixel(p);
	}

}