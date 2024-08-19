#include "Utils/Transform.h"

#include "glm/gtc/matrix_transform.hpp"
#include "Utils/Interaction.h"

namespace RT
{
	//-------------------------------------------Utility functions-------------------------------------

	Transform translate(const Vec3f &delta)
	{
		AMatrix4x4 trans = glm::translate(AMatrix4x4(1.0f), delta);
		AMatrix4x4 transInv = glm::translate(AMatrix4x4(1.0f), -delta);
		return Transform(trans, transInv);
	}

	Transform scale(Float x, Float y, Float z)
	{
		AMatrix4x4 trans = glm::scale(AMatrix4x4(1.0f), Vec3f(x, y, z));
		AMatrix4x4 transInv = glm::scale(AMatrix4x4(1.0f), Vec3f(1 / x, 1 / y, 1 / z));
		return Transform(trans, transInv);
	}

	Transform rotateX(Float theta)
	{
		AMatrix4x4 trans = glm::rotate(AMatrix4x4(1.0f), glm::radians(theta), Vec3f(1, 0, 0));
		AMatrix4x4 transInv = inverse(trans);
		return Transform(trans, transInv);
	}

	Transform rotateY(Float theta)
	{
		AMatrix4x4 trans = glm::rotate(AMatrix4x4(1.0f), glm::radians(theta), Vec3f(0, 1, 0));
		AMatrix4x4 transInv = inverse(trans);
		return Transform(trans, transInv);
	}

	Transform rotateZ(Float theta)
	{
		AMatrix4x4 trans = glm::rotate(AMatrix4x4(1.0f), glm::radians(theta), Vec3f(0, 0, 1));
		AMatrix4x4 transInv = inverse(trans);
		return Transform(trans, transInv);
	}

	Transform rotate(Float theta, const Vec3f &axis)
	{
		AMatrix4x4 trans = glm::rotate(AMatrix4x4(1.0f), glm::radians(theta), axis);
		AMatrix4x4 transInv = inverse(trans);
		return Transform(trans, transInv);
	}

	Transform lookAt(const Vec3f &pos, const Vec3f &look, const Vec3f &up)
	{
		AMatrix4x4 worldToCamera = glm::lookAt(pos, look, up);
		return Transform(worldToCamera, inverse(worldToCamera));
	}

	Transform orthographic(Float znear, Float zfar)
	{
		return scale(1, 1, 1 / (zfar - znear)) * translate(Vec3f(0, 0, -znear));
	}

	Transform perspective(Float fov, Float n, Float f)
	{
		// Perform projective divide for perspective projection
		AMatrix4x4 persp(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, f / (f - n), 1,
			0, 0, -f * n / (f - n), 0);
		// Scale canonical perspective view to specified field of view
		Float invTanAng = 1 / glm::tan(glm::radians(fov) / 2);
		return scale(invTanAng, invTanAng, 1) * Transform(persp);
	}

	//-------------------------------------------ATransform-------------------------------------

	Transform Transform::operator*(const Transform &t2) const
	{
		return Transform(mul(m_trans, t2.m_trans), mul(t2.m_transInv, m_transInv));
	}

	BBox3f Transform::operator()(const BBox3f &b) const
	{
		const Transform &mat = *this;
		BBox3f ret(mat(Vec3f(b.m_pMin.x, b.m_pMin.y, b.m_pMin.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMax.x, b.m_pMin.y, b.m_pMin.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMin.x, b.m_pMax.y, b.m_pMin.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMin.x, b.m_pMin.y, b.m_pMax.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMin.x, b.m_pMax.y, b.m_pMax.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMax.x, b.m_pMax.y, b.m_pMin.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMax.x, b.m_pMin.y, b.m_pMax.z), 1.0f));
		ret = unionBounds(ret, mat(Vec3f(b.m_pMax.x, b.m_pMax.y, b.m_pMax.z), 1.0f));
		return ret;
	}

	SurfaceInteraction Transform::operator()(const SurfaceInteraction &si) const
	{
		SurfaceInteraction ret;
		// Transform _p_ and _pError_ in _SurfaceInteraction_
		ret.p = (*this)(si.p, 1.0f);

		// Transform remaining members of _SurfaceInteraction_
		const Transform &trans = *this;
		ret.n = normalize(trans(si.n, 0.0f));
		ret.wo = normalize(trans(si.wo, 0.0f));
		ret.uv = si.uv;
		ret.shape = si.shape;
		ret.dpdu = trans(si.dpdu, 0.0f);
		ret.dpdv = trans(si.dpdv, 0.0f);
		ret.hitable = si.hitable;
		//    ret.n = Faceforward(ret.n, ret.shading.n);
		return ret;
	}

}