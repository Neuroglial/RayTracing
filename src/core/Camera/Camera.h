#pragma once

#include "Utils/Base.h"
#include "Object/Film.h"
#include "Utils/Transform.h"
#include "Object/Object.h"

namespace RT
{
	struct CameraSample
	{
		Vec2f pFilm;
	};

	inline std::ostream &operator<<(std::ostream &os, const CameraSample &cs) 
	{
		os << "[ pFilm: " << cs.pFilm << " ]";
		return os;
	}

	class Camera : public Object
	{
	public:
		typedef std::shared_ptr<Camera> ptr;

		// Camera Interface
		Camera() = default;
		Camera(const Transform &cameraToWorld, Film::ptr film);
		virtual ~Camera();

		virtual Float castingRay(const CameraSample &sample, Ray &ray) const = 0;

		//virtual spectrum We(const ARay &ray, APoint2f *pRaster2 = nullptr) const;
		//virtual void pdf_We(const ARay &ray, Float *pdfPos, Float *pdfDir) const;

		//virtual Spectrum Sample_Wi(const Interaction &ref, const APoint2f &u,
		//	Vec3f *wi, Float *pdf, APoint2f *pRaster, VisibilityTester *vis) const;

		virtual AClassType getClassType() const override { return AClassType::AECamera; }

		// Camera Public Data
		Transform m_cameraToWorld;
		Film::ptr m_film = nullptr;
	};

	class AProjectiveCamera : public Camera
	{
	public:
		typedef std::shared_ptr<AProjectiveCamera> ptr;

		AProjectiveCamera() = default;
		AProjectiveCamera(const Transform &cameraToWorld, const Transform &cameraToScreen, Film::ptr film)
			: Camera(cameraToWorld, film), m_cameraToScreen(cameraToScreen) { }

	protected:
		virtual void initialize();

	protected:
		Transform m_cameraToScreen, m_rasterToCamera;
		Transform m_screenToRaster, m_rasterToScreen;
	};

}

namespace RT
{

	class APerspectiveCamera final : public AProjectiveCamera
	{
	public:
		typedef std::shared_ptr<APerspectiveCamera> ptr;

		APerspectiveCamera(const PropertyTreeNode& node);
		APerspectiveCamera(const Transform& CameraToWorld, Float fov, Film::ptr film);

		virtual Float castingRay(const CameraSample& sample, Ray& ray) const override;

		//Spectrum We(const Ray &ray, Point2f *pRaster2 = nullptr) const;
		//void Pdf_We(const Ray &ray, Float *pdfPos, Float *pdfDir) const;
		//Spectrum Sample_Wi(const Interaction &ref, const Point2f &sample,
		//	Vector3f *wi, Float *pdf, Point2f *pRaster,
		//	VisibilityTester *vis) const;

		virtual void activate() override { initialize(); }

		virtual std::string toString() const override { return "PerspectiveCamera[]"; }

	protected:
		virtual void initialize() override;

	private:
		//Vec3f dxCamera, dyCamera;
		Float A;
	};
}