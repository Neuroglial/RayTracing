#ifndef ARTRANSFORM_H
#define ARTRANSFORM_H

#include "Utils/Base.h"
#include "Utils/Math.h"

namespace RT
{

	class Transform
	{
	public:
		typedef std::shared_ptr<Transform> ptr;

		Transform() : m_trans(AMatrix4x4(1.0f)), m_transInv(AMatrix4x4(1.0f)){}

		Transform(const Float mat[4][4])
		{
			//Note: column major matrix
			m_trans = AMatrix4x4(
				mat[0][0], mat[1][0], mat[2][0], mat[3][0],
				mat[0][1], mat[1][1], mat[2][1], mat[3][1],
				mat[0][2], mat[1][2], mat[2][2], mat[3][2],
				mat[0][3], mat[1][3], mat[2][3], mat[3][3]);
			m_transInv = inverse(m_trans);
		}

		Transform(const AMatrix4x4 &m) : m_trans(m), m_transInv(inverse(m)) {}
		Transform(const AMatrix4x4 &m, const AMatrix4x4 &mInv) : m_trans(m), m_transInv(mInv) {}

		friend Transform inverse(const Transform &t) { return Transform(t.m_transInv, t.m_trans); }
		friend Transform transpose(const Transform &t) { return Transform(transpose(t.m_trans), transpose(t.m_trans)); }

		bool operator==(const Transform &t) const { return t.m_trans == m_trans && t.m_transInv == m_transInv; }
		bool operator!=(const Transform &t) const { return t.m_trans != m_trans || t.m_transInv != m_transInv; }

		//Ray
		inline Ray operator()(const Ray &r) const;
		//Bounds
		BBox3f operator()(const BBox3f &b) const;
		//SurfaceInteraction
		SurfaceInteraction operator()(const SurfaceInteraction &si) const;
		//Vector
		template <typename T>
		inline Vec3<T> operator()(const Vec3<T> &p, const Float &w) const;

		bool isIdentity() const
		{
			return (
				m_trans[0][0] == 1.f && m_trans[1][0] == 0.f && m_trans[2][0] == 0.f && m_trans[3][0] == 0.f &&
				m_trans[0][1] == 0.f && m_trans[1][1] == 1.f && m_trans[2][1] == 0.f && m_trans[3][1] == 0.f &&
				m_trans[0][2] == 0.f && m_trans[1][2] == 0.f && m_trans[2][2] == 1.f && m_trans[3][2] == 0.f &&
				m_trans[0][3] == 0.f && m_trans[1][3] == 0.f && m_trans[2][3] == 0.f && m_trans[3][3] == 1.f);
		}

		const AMatrix4x4 &getMatrix() const { return m_trans; }
		const AMatrix4x4 &getInverseMatrix() const { return m_transInv; }

		Transform operator*(const Transform &t2) const;

	private:
		AMatrix4x4 m_trans, m_transInv;
	};

	Transform translate(const Vec3f &delta);
	Transform scale(Float x, Float y, Float z);
	Transform rotateX(Float theta);
	Transform rotateY(Float theta);
	Transform rotateZ(Float theta);
	Transform rotate(Float theta, const Vec3f &axis);
	Transform lookAt(const Vec3f &pos, const Vec3f &look, const Vec3f &up);
	Transform orthographic(Float znear, Float zfar);
	Transform perspective(Float fov, Float znear, Float zfar);

	template <typename T>
	inline Vec3<T> Transform::operator()(const Vec3<T> &p, const Float &w) const
	{
		//Note: w == 1.f -> point, w == 0.f -> vector
		glm::vec<4, Float> ret = m_trans * glm::vec<4, Float>(p.x, p.y, p.z, w);
		if (w == 0.f)
			return Vec3<T>(ret.x, ret.y, ret.z);

		CHECK_NE(ret.w, 0);
		if (ret.w == 1)
			return Vec3<T>(ret.x, ret.y, ret.z);
		else
			return Vec3<T>(ret.x, ret.y, ret.z) / ret.w;
	}

	inline Ray Transform::operator()(const Ray &r) const
	{
		Vec3f o = (*this)(r.m_origin, 1.0f);
		Vec3f d = (*this)(r.m_dir, 0.0f);
		Float tMax = r.m_tMax;

		return Ray(o, d, tMax);
	}

}

#endif