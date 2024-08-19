#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Shape/Shape.h"
#include "Utils/Color.h"
#include "Utils/Transform.h"
#include "Utils/Interaction.h"
#include "Object/Object.h"

namespace RT
{
	enum class ALightFlags : int
	{
		ALightDeltaPosition = 1,
		ALightDeltaDirection = 2,
		ALightArea = 4,
		ALightInfinite = 8
	};

	inline bool isDeltaLight(int flags)
	{
		return flags & (int)ALightFlags::ALightDeltaPosition 
			|| flags & (int)ALightFlags::ALightDeltaDirection;
	}

	class Light : public Object
	{
	public:
		typedef std::shared_ptr<Light> ptr;

		Light(const PropertyList &props);
		Light(int flags, const Transform &LightToWorld, int nSamples = 1);

		virtual ~Light();

		virtual Spectrum power() const = 0;

		virtual void preprocess(const Scene &scene) {}

		virtual Spectrum sample_Li(const Interaction &ref, const Vec2f &u,
			Vec3f &wi, Float &pdf, VisibilityTester &vis) const = 0;

		virtual Float pdf_Li(const Interaction &, const Vec3f &) const = 0;

		//Emission
		virtual Spectrum Le(const Ray &r) const;

		virtual Spectrum sample_Le(const Vec2f &u1, const Vec2f &u2, Ray &ray,
			Vec3f &nLight, Float &pdfPos, Float &pdfDir) const = 0;

		virtual void pdf_Le(const Ray &, const Vec3f &, Float &pdfPos, Float &pdfDir) const = 0;

		virtual AClassType getClassType() const override { return AClassType::AELight; }

		int m_flags;
		int m_nSamples;

	protected:
		Transform m_lightToWorld, m_worldToLight;
	};

	class VisibilityTester final
	{
	public:
		VisibilityTester() {}
		VisibilityTester(const Interaction &p0, const Interaction &p1)
			: m_p0(p0), m_p1(p1) {}

		const Interaction &P0() const { return m_p0; }
		const Interaction &P1() const { return m_p1; }

		bool unoccluded(const Scene &scene) const;

		Spectrum tr(const Scene &scene, Sampler &sampler) const;

	private:
		Interaction m_p0, m_p1;
	};

	class AreaLight : public Light
	{
	public:
		typedef std::shared_ptr<AreaLight> ptr;

		AreaLight(const PropertyList &props);
		AreaLight(const Transform &lightToWorld, int nSamples);
		virtual Spectrum L(const Interaction &intr, const Vec3f &w) const = 0;
	};

}

namespace RT
{
	class ADiffuseAreaLight final : public AreaLight
	{
	public:
		typedef std::shared_ptr<ADiffuseAreaLight> ptr;

		ADiffuseAreaLight(const PropertyTreeNode& node);

		ADiffuseAreaLight(const Transform& lightToWorld, const Spectrum& Le, int nSamples,
			Shape* shape, bool twoSided = false);

		virtual Spectrum L(const Interaction& intr, const Vec3f& w) const override
		{
			return (m_twoSided || dot(intr.n, w) > 0) ? m_Lemit : Spectrum(0.f);
		}

		virtual Spectrum power() const override;

		virtual Spectrum sample_Li(const Interaction& ref, const Vec2f& u, Vec3f& wo,
			Float& pdf, VisibilityTester& vis) const override;

		virtual Float pdf_Li(const Interaction&, const Vec3f&) const override;

		virtual Spectrum sample_Le(const Vec2f& u1, const Vec2f& u2, Ray& ray,
			Vec3f& nLight, Float& pdfPos, Float& pdfDir) const override;

		virtual void pdf_Le(const Ray&, const Vec3f&, Float& pdfPos, Float& pdfDir) const override;

		virtual std::string toString() const override { return "DiffuseAreaLight[]"; }

		virtual void setParent(Object* parent) override;

	protected:

		Spectrum m_Lemit;
		Shape* m_shape;
		// Added after book publication: by default, DiffuseAreaLights still
		// only emit in the hemimsphere around the surface normal.  However,
		// this behavior can now be overridden to give emission on both sides.
		bool m_twoSided;
		Float m_area;
	};
}