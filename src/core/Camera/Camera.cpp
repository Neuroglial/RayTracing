#include "Camera/Camera.h"

namespace RT
{
	//-------------------------------------------ACamera-------------------------------------

	Camera::Camera(const Transform &cameraToWorld, Film::ptr film)
		:m_cameraToWorld(cameraToWorld), m_film(film) {}

	Camera::~Camera() {}

	//-------------------------------------------AProjectiveCamera-------------------------------------

	void AProjectiveCamera::initialize()
	{
		// Compute projective camera screen transformations
		BBox2f screen;
		auto res = m_film->getResolution();
		Float frame = (Float)(res.x) / res.y;
		if (frame > 1.f)
		{
			screen.m_pMin.x = -frame;
			screen.m_pMax.x = frame;
			screen.m_pMin.y = -1.f;
			screen.m_pMax.y = 1.f;
		}
		else
		{
			screen.m_pMin.x = -1.f;
			screen.m_pMax.x = 1.f;
			screen.m_pMin.y = -1.f / frame;
			screen.m_pMax.y = 1.f / frame;
		}

		m_screenToRaster = scale(res.x, res.y, 1) *
			scale(1 / (screen.m_pMax.x - screen.m_pMin.x),
				1 / (screen.m_pMin.y - screen.m_pMax.y), 1) *
			translate(Vec3f(-screen.m_pMin.x, -screen.m_pMax.y, 0));
		m_rasterToScreen = inverse(m_screenToRaster);
		m_rasterToCamera = inverse(m_cameraToScreen) * m_rasterToScreen;
	}
}

namespace RT
{
	AURORA_REGISTER_CLASS(APerspectiveCamera, "Perspective")

		APerspectiveCamera::APerspectiveCamera(const PropertyTreeNode& node)
	{
		const auto props = node.getPropertyList();
		Float _fov = props.getFloat("Fov");
		auto _eye = props.getVector3f("Eye");
		auto _focus = props.getVector3f("Focus");
		auto _up = props.getVector3f("WorldUp", Vec3f(0.f, 1.f, 0.f));
		m_cameraToWorld = inverse(lookAt(_eye, _focus, _up));
		m_cameraToScreen = perspective(_fov, 1e-2f, 1000.f);

		//Film
		{
			const auto& filmNode = node.getPropertyChild("Film");
			m_film = Film::ptr(static_cast<Film*>(ObjectFactory::createInstance(
				filmNode.getTypeName(), filmNode)));
		}

		activate();
	}

	APerspectiveCamera::APerspectiveCamera(const Transform& cameraToWorld, Float fov, Film::ptr film)
		: AProjectiveCamera(cameraToWorld, perspective(fov, 1e-2f, 1000.f), film)
	{
		initialize();
	}

	void APerspectiveCamera::initialize()
	{
		// Compute image plane bounds at $z=1$ for _PerspectiveCamera_
		Vec2i res = m_film->getResolution();
		Vec3f pMin = m_rasterToCamera(Vec3f(0, 0, 0), 1.0f);
		Vec3f pMax = m_rasterToCamera(Vec3f(res.x, res.y, 0), 1.0f);
		pMin /= pMin.z;
		pMax /= pMax.z;
		A = glm::abs((pMax.x - pMin.x) * (pMax.y - pMin.y));

		AProjectiveCamera::initialize();
	}

	Float APerspectiveCamera::castingRay(const CameraSample& sample, Ray& ray) const
	{
		// Compute raster and camera sample positions
		Vec3f pFilm = Vec3f(sample.pFilm.x, sample.pFilm.y, 0);
		Vec3f pCamera = m_rasterToCamera(pFilm, 1.0f);
		ray = Ray(Vec3f(0, 0, 0), normalize(Vec3f(pCamera)));
		ray = m_cameraToWorld(ray);
		return 1.f;
	}
}