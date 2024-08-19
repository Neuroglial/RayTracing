#pragma once

#include "Utils/Base.h"
#include "Utils/Interaction.h"
#include "Utils/Math.h"
#include "Utils/Color.h"

namespace RT
{
	enum ABxDFType
	{
		BSDF_REFLECTION = 1 << 0,
		BSDF_TRANSMISSION = 1 << 1,
		BSDF_DIFFUSE = 1 << 2,
		BSDF_GLOSSY = 1 << 3,
		BSDF_SPECULAR = 1 << 4,
		BSDF_ALL = BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR | BSDF_REFLECTION | BSDF_TRANSMISSION,
	};

	Float frDielectric(Float cosThetaI, Float etaI, Float etaT);

	inline bool sameHemisphere(const Vec3f &w, const Vec3f &wp) { return w.z * wp.z > 0; }

	class BSDF
	{
	public:
		typedef std::shared_ptr<BSDF> ptr;

		// BSDF Public Methods
		BSDF(const SurfaceInteraction &si, Float eta = 1)
			: m_eta(eta), m_ns(si.n), m_ss(normalize(si.dpdu)), m_ts(cross(m_ns, m_ss)) {}

		~BSDF() = default;

		void add(BxDF *b) 
		{
			CHECK_LT(m_nBxDFs, NumMaxBxDFs);
			m_bxdfs[m_nBxDFs++] = b; 
		}

		int numComponents(ABxDFType flags = BSDF_ALL) const;

		Vec3f worldToLocal(const Vec3f &v) const
		{
			return Vec3f(dot(v, m_ss), dot(v, m_ts), dot(v, m_ns));
		}

		Vec3f localToWorld(const Vec3f &v) const
		{
			return Vec3f(
				m_ss.x * v.x + m_ts.x * v.y + m_ns.x * v.z,
				m_ss.y * v.x + m_ts.y * v.y + m_ns.y * v.z,
				m_ss.z * v.x + m_ts.z * v.y + m_ns.z * v.z);
		}

		Spectrum f(const Vec3f &woW, const Vec3f &wiW, ABxDFType flags = BSDF_ALL) const;

		Spectrum sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &u, Float &pdf,
			ABxDFType &sampledType, ABxDFType type = BSDF_ALL) const;

		Float pdf(const Vec3f &wo, const Vec3f &wi, ABxDFType flags = BSDF_ALL) const;

		//Refractive index
		const Float m_eta;

	private:
		int m_nBxDFs = 0;
		const Vec3f m_ns, m_ss, m_ts;

		static constexpr int NumMaxBxDFs = 8;
		BxDF *m_bxdfs[NumMaxBxDFs];
	};

	class BxDF
	{
	public:
		BxDF(ABxDFType type) : m_type(type) {}

		virtual ~BxDF() = default;

		bool matchesFlags(ABxDFType t) const { return (m_type & t) == m_type; }

		virtual Spectrum f(const Vec3f &wo, const Vec3f &wi) const = 0;
		virtual Spectrum sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
			Float &pdf, ABxDFType &sampledType) const;

		virtual Float pdf(const Vec3f &wo, const Vec3f &wi) const;

		// BxDF Public Data
		const ABxDFType m_type;
	};

	class AFresnel
	{
	public:
		virtual ~AFresnel() = default;
		virtual Spectrum evaluate(Float cosI) const = 0;
	};

	class AFresnelDielectric : public AFresnel
	{
	public:
		AFresnelDielectric(Float etaI, Float etaT) : m_etaI(etaI), m_etaT(etaT) {}

		virtual Spectrum evaluate(Float cosThetaI) const override
		{
			return frDielectric(cosThetaI, m_etaI, m_etaT);
		}

	private:
		Float m_etaI, m_etaT;
	};

	class AFresnelNoOp : public AFresnel
	{
	public:
		virtual Spectrum evaluate(Float) const override { return Spectrum(1.); }
	};

	class ALambertianReflection : public BxDF
	{
	public:
		// LambertianReflection Public Methods
		ALambertianReflection(const Spectrum &R)
			: BxDF(ABxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), m_R(R) {}

		virtual Spectrum f(const Vec3f &wo, const Vec3f &wi) const override;

	private:
		// LambertianReflection Private Data
		const Spectrum m_R;
	};

	class ASpecularReflection : public BxDF
	{
	public:
		// SpecularReflection Public Methods
		ASpecularReflection(const Spectrum &R, AFresnel *fresnel)
			: BxDF(ABxDFType(BSDF_REFLECTION | BSDF_SPECULAR)),
			m_R(R),
			m_fresnel(fresnel) {}

		virtual Spectrum f(const Vec3f &wo, const Vec3f &wi) const override { return Spectrum(0.f); }

		virtual Spectrum sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
			Float &pdf, ABxDFType &sampledType) const override;

		virtual Float pdf(const Vec3f &wo, const Vec3f &wi) const override { return 0.f; }

	private:
		// SpecularReflection Private Data
		const Spectrum m_R;
		const AFresnel *m_fresnel;
	};

	class ASpecularTransmission : public BxDF
	{
	public:
		// SpecularTransmission Public Methods
		ASpecularTransmission(const Spectrum &T, Float etaA, Float etaB, ATransportMode mode)
			: BxDF(ABxDFType(BSDF_TRANSMISSION | BSDF_SPECULAR)), m_T(T), m_etaA(etaA),
			m_etaB(etaB), m_fresnel(etaA, etaB), m_mode(mode) {}

		virtual Spectrum f(const Vec3f &wo, const Vec3f &wi) const override { return Spectrum(0.f); }

		virtual Spectrum sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
			Float &pdf, ABxDFType &sampledType) const override;

		virtual Float pdf(const Vec3f &wo, const Vec3f &wi) const override { return 0.f; }

	private:
		const Spectrum m_T;
		const Float m_etaA, m_etaB;
		const AFresnelDielectric m_fresnel;
		const ATransportMode m_mode;
	};

}