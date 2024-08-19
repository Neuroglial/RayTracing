#include "Render/BSDF.h"

#include "Render/Sampler.h"

namespace RT
{
	//-------------------------------------------ABSDF-------------------------------------

	int BSDF::numComponents(ABxDFType flags) const
	{
		int num = 0;
		for (int i = 0; i < m_nBxDFs; ++i)
		{
			if (m_bxdfs[i]->matchesFlags(flags))
			{
				++num;
			}
		}
		return num;
	}

	Spectrum BSDF::f(const Vec3f &woW, const Vec3f &wiW, ABxDFType flags) const
	{
		Vec3f wi = worldToLocal(wiW), wo = worldToLocal(woW);
		if (wo.z == 0)
			return 0.f;

		bool reflect = dot(wiW, m_ns) * dot(woW, m_ns) > 0;
		Spectrum f(0.f);
		for (int i = 0; i < m_nBxDFs; ++i)
		{
			if (m_bxdfs[i]->matchesFlags(flags) &&
				((reflect && (m_bxdfs[i]->m_type & BSDF_REFLECTION)) ||
				(!reflect && (m_bxdfs[i]->m_type & BSDF_TRANSMISSION))))
			{
				f += m_bxdfs[i]->f(wo, wi);
			}
		}
		return f;
	}

	Spectrum BSDF::sample_f(const Vec3f &woWorld, Vec3f &wiWorld, const Vec2f &u,
		Float &pdf, ABxDFType &sampledType, ABxDFType type) const
	{
		// Choose which _BxDF_ to sample
		int matchingComps = numComponents(type);
		if (matchingComps == 0) 
		{
			pdf = 0;
			if (sampledType)
			{
				sampledType = ABxDFType(0);
			}
			return Spectrum(0);
		}
		int comp = glm::min((int)glm::floor(u[0] * matchingComps), matchingComps - 1);

		// Get _BxDF_ pointer for chosen component
		BxDF *bxdf = nullptr;
		int count = comp;
		for (int i = 0; i < m_nBxDFs; ++i) 
		{
			if (m_bxdfs[i]->matchesFlags(type) && count-- == 0) 
			{
				bxdf = m_bxdfs[i];
				break;
			}
		}
		CHECK(bxdf != nullptr);

		// Remap _BxDF_ sample _u_ to $[0,1)^2$
		Vec2f uRemapped(glm::min(u[0] * matchingComps - comp, aOneMinusEpsilon), u[1]);

		// Sample chosen _BxDF_
		Vec3f wi, wo = worldToLocal(woWorld);
		if (wo.z == 0)
		{
			return 0.f;
		}
		
		pdf = 0;
		if (sampledType)
		{
			sampledType = bxdf->m_type;
		}
		Spectrum f = bxdf->sample_f(wo, wi, uRemapped, pdf, sampledType);

		if (pdf == 0) 
		{
			if (sampledType)
			{
				sampledType = ABxDFType(0);
			}
			return 0;
		}

		wiWorld = localToWorld(wi);

		// Compute overall PDF with all matching _BxDF_s
		if (!(bxdf->m_type & BSDF_SPECULAR) && matchingComps > 1)
		{
			for (int i = 0; i < m_nBxDFs; ++i)
			{
				if (m_bxdfs[i] != bxdf && m_bxdfs[i]->matchesFlags(type))
					pdf += m_bxdfs[i]->pdf(wo, wi);
			}
		}
		if (matchingComps > 1)
		{
			pdf /= matchingComps;
		}

		// Compute value of BSDF for sampled direction
		if (!(bxdf->m_type & BSDF_SPECULAR)) 
		{
			bool reflect = dot(wiWorld, m_ns) * dot(woWorld, m_ns) > 0;
			f = 0.;
			for (int i = 0; i < m_nBxDFs; ++i)
			{
				if (m_bxdfs[i]->matchesFlags(type) &&
					((reflect && (m_bxdfs[i]->m_type & BSDF_REFLECTION)) ||
					(!reflect && (m_bxdfs[i]->m_type & BSDF_TRANSMISSION))))
				{
					f += m_bxdfs[i]->f(wo, wi);
				}
			}
		}
	
		return f;
	}

	Float BSDF::pdf(const Vec3f &woWorld, const Vec3f &wiWorld, ABxDFType flags) const
	{
		if (m_nBxDFs == 0)
		{
			return 0.f;
		}

		Vec3f wo = worldToLocal(woWorld), wi = worldToLocal(wiWorld);

		if (wo.z == 0)
		{
			return 0.;
		}

		Float pdf = 0.f;
		int matchingComps = 0;
		for (int i = 0; i < m_nBxDFs; ++i) 
		{
			if (m_bxdfs[i]->matchesFlags(flags))
			{
				++matchingComps;
				pdf += m_bxdfs[i]->pdf(wo, wi);
			}
		}
		Float v = matchingComps > 0 ? pdf / matchingComps : 0.f;
		return v;
	}

	//-------------------------------------------ABxDF-------------------------------------

	Spectrum BxDF::sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
		Float &pdf, ABxDFType &sampledType) const
	{
		// Cosine-sample the hemisphere, flipping the direction if necessary
		wi = cosineSampleHemisphere(sample);
		if (wo.z < 0)
		{
			wi.z *= -1;
		}

		pdf = this->pdf(wo, wi);

		return f(wo, wi);
	}

	Float BxDF::pdf(const Vec3f &wo, const Vec3f &wi) const
	{
		return sameHemisphere(wo, wi) ? glm::abs(wi.z) * InvPi : 0;
	}

	//-------------------------------------------ALambertianReflection-------------------------------------

	Spectrum ALambertianReflection::f(const Vec3f &wo, const Vec3f &wi) const
	{
		return m_R * InvPi;
	}

	//-------------------------------------------ASpecularReflection-------------------------------------

	Spectrum ASpecularReflection::sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
		Float &pdf, ABxDFType &sampledType) const
	{
		wi = Vec3f(-wo.x, -wo.y, wo.z);
		pdf = 1;
		return m_fresnel->evaluate(wi.z) * m_R / glm::abs(wi.z);
	}

	//-------------------------------------------ASpecularTransmission-------------------------------------

	Spectrum ASpecularTransmission::sample_f(const Vec3f &wo, Vec3f &wi, const Vec2f &sample,
		Float &pdf, ABxDFType &sampledType) const
	{
		// Figure out which $\eta$ is incident and which is transmitted
		bool entering = (wo.z) > 0;
		Float etaI = entering ? m_etaA : m_etaB;
		Float etaT = entering ? m_etaB : m_etaA;

		// Compute ray direction for specular transmission
		if (!refract(wo, faceforward(Vec3f(0, 0, 1), wo), etaI / etaT, wi))
			return 0;

		pdf = 1;
		Spectrum ft = m_T * (Spectrum(1.) - m_fresnel.evaluate(wi.z));

		// Account for non-symmetry with transmission to different medium
		if (m_mode == ATransportMode::aRadiance)
			ft *= (etaI * etaI) / (etaT * etaT);
		return ft / glm::abs(wi.z);
	}

	//-------------------------------------------Utility function-------------------------------------

	Float frDielectric(Float cosThetaI, Float etaI, Float etaT)
	{
		cosThetaI = clamp(cosThetaI, -1, 1);
		// Potentially swap indices of refraction
		bool entering = cosThetaI > 0.f;
		if (!entering)
		{
			std::swap(etaI, etaT);
			cosThetaI = glm::abs(cosThetaI);
		}

		// Compute _cosThetaT_ using Snell's law
		Float sinThetaI = glm::sqrt(glm::max((Float)0, 1 - cosThetaI * cosThetaI));
		Float sinThetaT = etaI / etaT * sinThetaI;

		// Handle total internal reflection
		if (sinThetaT >= 1)
			return 1;
		Float cosThetaT = glm::sqrt(glm::max((Float)0, 1 - sinThetaT * sinThetaT));
		Float Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) / ((etaT * cosThetaI) + (etaI * cosThetaT));
		Float Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) / ((etaI * cosThetaI) + (etaT * cosThetaT));
		return (Rparl * Rparl + Rperp * Rperp) / 2;
	}
}