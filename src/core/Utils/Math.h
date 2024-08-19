#pragma once

#include "Utils/Base.h"

#include "glm/common.hpp"
#include "glm/gtc/quaternion.hpp"

#include <ctime>
#include <iterator>

namespace RT
{
	typedef unsigned int uint;

	template <typename T>
	inline bool isNaN(const T x) { return std::isnan(x); }

	template <>
	inline bool isNaN(const int x) { return false; }

	//-------------------------------------------Vector/Point/Normal-------------------------------------

	template<typename T>
	using Vec2 = glm::vec<2, T>;

	template<typename T>
	using Vec3 = glm::vec<3, T>;

	typedef Vec2<Float> Vec2f;
	typedef Vec2<int> Vec2i;
	typedef Vec3<Float> Vec3f;
	typedef Vec3<int> Vec3i;

	template <typename T>
	inline std::ostream &operator<<(std::ostream &os, const Vec2<T> &v)
	{
		os << "[ " << v.x << ", " << v.y << " ]";
		return os;
	}

	template <typename T>
	inline std::ostream &operator<<(std::ostream &os, const Vec3<T> &v)
	{
		os << "[ " << v.x << ", " << v.y << ", " << v.z << " ]";
		return os;
	}


	//-------------------------------------------Matrix/Quaterion-------------------------------------

	using AMatrix4x4 = glm::mat<4, 4, Float>;
	using AQuaterion = glm::qua<Float>;

	inline AMatrix4x4 transpose(const AMatrix4x4 &m) { return glm::transpose(m); }
	inline AMatrix4x4 mul(const AMatrix4x4 &m1, const AMatrix4x4 &m2) { return m1 * m2; }
	inline AMatrix4x4 inverse(const AMatrix4x4 &m) { return glm::inverse(m); }

	inline AMatrix4x4 toMatrix4x4(const AQuaterion &q) { return glm::mat4_cast(q); }

	//-------------------------------------------BBox2/BBox3-------------------------------------

	template<typename T>
	class BBox2
	{
	public:

		BBox2()
		{
			T minNum = std::numeric_limits<T>::lowest();
			T maxNum = std::numeric_limits<T>::max();
			m_pMin = Vec2<T>(maxNum, maxNum);
			m_pMax = Vec2<T>(minNum, minNum);
		}

		explicit BBox2(const Vec2<T> &p) : m_pMin(p), m_pMax(p) {}

		BBox2(const Vec2<T> &p1, const Vec2<T> &p2)
		{
			m_pMin = Vec2<T>(glm::min(p1.x, p2.x), glm::min(p1.y, p2.y));
			m_pMax = Vec2<T>(glm::max(p1.x, p2.x), glm::max(p1.y, p2.y));
		}

		template <typename U>
		explicit operator BBox2<U>() const
		{
			return BBox2<U>((Vec2<U>)m_pMin, (Vec2<U>)m_pMax);
		}

		Vec2<T> diagonal() const { return m_pMax - m_pMin; }

		T area() const
		{
			Vec2<T> d = m_pMax - m_pMin;
			return (d.x * d.y);
		}

		int maximumExtent() const {
			Vec2<T> diag = diagonal();
			if (diag.x > diag.y)
				return 0;
			else
				return 1;
		}

		inline Vec2<T> &operator[](int i) 
		{ 
			DCHECK(i == 0 || i == 1);
			return (i == 0) ? m_pMin : m_pMax; 
		}

		inline const Vec2<T> &operator[](int i) const 
		{
			DCHECK(i == 0 || i == 1);
			return (i == 0) ? m_pMin : m_pMax; 
		}

		bool operator==(const BBox2<T> &b) const { return b.m_pMin == pMin && b.m_pMax == pMax; }
		bool operator!=(const BBox2<T> &b) const { return b.m_pMin != pMin || b.m_pMax != pMax; }

		Vec2<T> lerp(const Vec2f &t) const
		{
			return Vec2<T>(Aurora::lerp(t.x, m_pMin.x, m_pMax.x), Aurora::lerp(t.y, m_pMin.y, m_pMax.y));
		}

		Vec2<T> offset(const Vec2<T> &p) const
		{
			Vec2<T> o = p - m_pMin;
			if (m_pMax.x > m_pMin.x) o.x /= m_pMax.x - m_pMin.x;
			if (m_pMax.y > m_pMin.y) o.y /= m_pMax.y - m_pMin.y;
			return o;
		}

		void boundingSphere(Vec2<T> *c, Float *rad) const
		{
			*c = (m_pMin + m_pMax) / 2;
			*rad = Inside(*c, *this) ? Distance(*c, m_pMax) : 0;
		}

		friend std::ostream &operator<<(std::ostream &os, const BBox2<T> &b)
		{
			os << "[ " << b.m_pMin << " - " << b.m_pMax << " ]";
			return os;
		}

	public:
		Vec2<T> m_pMin, m_pMax;

	};

	class Ray;
	template<typename T>
	class BBox3
	{
	public:

		BBox3()
		{
			T minNum = std::numeric_limits<T>::lowest();
			T maxNum = std::numeric_limits<T>::max();
			m_pMin = Vec3<T>(maxNum, maxNum, maxNum);
			m_pMax = Vec3<T>(minNum, minNum, minNum);
		}

		explicit BBox3(const Vec3<T> &p) : m_pMin(p), m_pMax(p) {}

		BBox3(const Vec3<T> &p1, const Vec3<T> &p2) :
			m_pMin(glm::min(p1.x, p2.x), glm::min(p1.y, p2.y), glm::min(p1.z, p2.z)),
			m_pMax(glm::max(p1.x, p2.x), glm::max(p1.y, p2.y), glm::max(p1.z, p2.z)) {}

		Vec3<T> &operator[](int i) { return (i == 0) ? m_pMin : m_pMax; }
		const Vec3<T> &operator[](int i) const { return (i == 0) ? m_pMin : m_pMax; }

		bool operator==(const BBox3<T> &b) const { return b.m_pMin == m_pMin && b.m_pMax == m_pMax; }
		bool operator!=(const BBox3<T> &b) const { return b.m_pMin != m_pMin || b.m_pMax != m_pMax; }

		Vec3<T> corner(int cor) const
		{
			DCHECK(cor >= 0 && cor < 8);
			return Vec3<T>((*this)[(cor & 1)].x, (*this)[(cor & 2) ? 1 : 0].y, (*this)[(cor & 4) ? 1 : 0].z);
		}

		Vec3<T> diagonal() const { return m_pMax - m_pMin; }

		T surfaceArea() const
		{
			Vec3<T> d = diagonal();
			return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
		}

		T volume() const
		{
			Vec3<T> d = diagonal();
			return d.x * d.y * d.z;
		}

		int maximumExtent() const
		{
			Vec3<T> d = diagonal();
			if (d.x > d.y && d.x > d.z)
				return 0;
			else if (d.y > d.z)
				return 1;
			else
				return 2;
		}

		Vec3<T> lerp(const Vec3f &t) const
		{
			return Vec3<T>(
				Aurora::lerp(t.x, m_pMin.x, m_pMax.x),
				Aurora::lerp(t.y, m_pMin.y, m_pMax.y),
				Aurora::lerp(t.z, m_pMin.z, m_pMax.z));
		}

		Vec3<T> offset(const Vec3<T> &p) const
		{
			Vec3<T> o = p - m_pMin;
			if (m_pMax.x > m_pMin.x) o.x /= m_pMax.x - m_pMin.x;
			if (m_pMax.y > m_pMin.y) o.y /= m_pMax.y - m_pMin.y;
			if (m_pMax.z > m_pMin.z) o.z /= m_pMax.z - m_pMin.z;
			return o;
		}

		void boundingSphere(Vec3<T> *center, Float *radius) const
		{
			*center = (m_pMin + m_pMax) / 2;
			*radius = Inside(*center, *this) ? Distance(*center, m_pMax) : 0;
		}

		template <typename U>
		explicit operator BBox3<U>() const
		{
			return BBox3<U>((Vec3<U>)m_pMin, (Vec3<U>)m_pMax);
		}

		bool hit(const Ray &ray, Float &hitt0, Float &hitt1) const;
		inline bool hit(const Ray &ray, const Vec3f &invDir, const int dirIsNeg[3]) const;

		friend std::ostream &operator<<(std::ostream &os, const BBox3<T> &b)
		{
			os << "[ " << b.m_pMin << " - " << b.m_pMax << " ]";
			return os;
		}

	public:
		Vec3<T> m_pMin, m_pMax;
	};

	typedef BBox2<Float> BBox2f;
	typedef BBox2<int> BBox2i;
	typedef BBox3<Float> BBox3f;
	typedef BBox3<int> BBox3i;

	class BBox2iIterator : public std::forward_iterator_tag 
	{
	public:
		BBox2iIterator(const BBox2i &b, const Vec2i &pt)
			: p(pt), bounds(&b) {}

		BBox2iIterator operator++() 
		{
			advance();
			return *this;
		}

		BBox2iIterator operator++(int) 
		{
			BBox2iIterator old = *this;
			advance();
			return old;
		}

		bool operator==(const BBox2iIterator &bi) const 
		{
			return p == bi.p && bounds == bi.bounds;
		}

		bool operator!=(const BBox2iIterator &bi) const 
		{
			return p != bi.p || bounds != bi.bounds;
		}

		Vec2i operator*() const { return p; }

	private:
		void advance() 
		{
			++p.x;
			if (p.x == bounds->m_pMax.x) 
			{
				p.x = bounds->m_pMin.x;
				++p.y;
			}
		}

		Vec2i p;
		const BBox2i *bounds;
	};

	inline BBox2iIterator begin(const BBox2i &b) 
	{
		return BBox2iIterator(b, b.m_pMin);
	}

	inline BBox2iIterator end(const BBox2i &b) 
	{
		// Normally, the ending point is at the minimum x value and one past
		// the last valid y value.
		Vec2i pEnd(b.m_pMin.x, b.m_pMax.y);
		// However, if the bounds are degenerate, override the end point to
		// equal the start point so that any attempt to iterate over the bounds
		// exits out immediately.
		if (b.m_pMin.x >= b.m_pMax.x || b.m_pMin.y >= b.m_pMax.y)
			pEnd = b.m_pMin;
		return BBox2iIterator(b, pEnd);
	}

	//-------------------------------------------ARay-------------------------------------

	class Ray
	{
	public:

		Ray() : m_tMax(Infinity) {}

		Ray(const Vec3f &o, const Vec3f &d, Float tMax = Infinity)
			: m_origin(o), m_dir(normalize(d)), m_tMax(tMax) {}

		const Vec3f &origin() const { return m_origin; }
		const Vec3f &direction() const { return m_dir; }

		Vec3f operator()(Float t) const { return m_origin + m_dir * t; }

		friend std::ostream &operator<<(std::ostream &os, const Ray &r)
		{
			os << "[o=" << r.m_origin << ", d=" << r.m_dir << ", tMax=" << r.m_tMax << "]";
			return os;
		}

	public:
		Vec3f m_origin;
		Vec3f m_dir;
		mutable Float m_tMax;
	};

	//-------------------------------------------Defnition-------------------------------------

	template <typename T>
	inline Float dot(const Vec2<T> &v1, const Vec2<T> &v2) { return glm::dot(v1, v2); }

	template <typename T>
	inline Float absDot(const Vec2<T> &v1, const Vec2<T> &v2) { return glm::abs(dot(v1, v2)); }

	template <typename T>
	inline Vec2<T> normalize(const Vec2<T> &v) { return glm::normalize(v); }

	template <typename T>
	Vec2<T> abs(const Vec2<T> &v) { return glm::abs(v); }

	template <typename T>
	inline Float length(const Vec3<T> &p) { return glm::length(p); }

	template <typename T>
	inline Float lengthSquared(const Vec3<T> &p) { return glm::dot(p, p); }

	template <typename T>
	inline Float distance(const Vec3<T> &p1, const Vec3<T> &p2) { return glm::length(p1 - p2); }

	template <typename T>
	inline Float distanceSquared(const Vec3<T> &p1, const Vec3<T> &p2) { return glm::dot(p1 - p2, p1 - p2); }

	template <typename T>
	inline Vec3<T> lerp(Float t, const Vec3<T> &p0, const Vec3<T> &p1) { return (1 - t) * p0 + t * p1; }

	template <typename T>
	inline Vec3<T> min(const Vec3<T> &p1, const Vec3<T> &p2)
	{
		return Vec3<T>(glm::min(p1.x, p2.x), glm::min(p1.y, p2.y), glm::min(p1.z, p2.z));
	}

	template <typename T>
	inline Vec3<T> max(const Vec3<T> &p1, const Vec3<T> &p2)
	{
		return Vec3<T>(glm::max(p1.x, p2.x), glm::max(p1.y, p2.y), glm::max(p1.z, p2.z));
	}

	template <typename T>
	inline Vec3<T> floor(const Vec3<T> &p)
	{
		return Vec3<T>(glm::floor(p.x), glm::floor(p.y), glm::floor(p.z));
	}

	template <typename T>
	inline Vec3<T> ceil(const Vec3<T> &p)
	{
		return Vec3<T>(glm::ceil(p.x), glm::ceil(p.y), glm::ceil(p.z));
	}

	template <typename T>
	inline Vec3<T> abs(const Vec3<T> &p)
	{
		return Vec3<T>(glm::abs(p.x), glm::abs(p.y), glm::abs(p.z));
	}

	template <typename T>
	inline Float distance(const Vec2<T> &p1, const Vec2<T> &p2) { return glm::length(p1, p2); }

	template <typename T>
	inline Float distanceSquared(const Vec2<T> &p1, const Vec2<T> &p2) { return glm::dot(p1 - p2, p1 - p2); }

	template <typename T>
	inline Vec2<T> floor(const Vec2<T> &p) { return Vec2<T>(glm::floor(p.x), glm::floor(p.y)); }

	template <typename T>
	inline Vec2<T> ceil(const Vec2<T> &p) { return Vec2<T>(glm::ceil(p.x), glm::ceil(p.y)); }

	template <typename T>
	inline Vec2<T> lerp(Float t, const Vec2<T> &v0, const Vec2<T> &v1) { return (1 - t) * v0 + t * v1; }

	template <typename T>
	inline Vec2<T> min(const Vec2<T> &pa, const Vec2<T> &pb) { return Vec2<T>(glm::min(pa.x, pb.x), glm::min(pa.y, pb.y)); }

	template <typename T>
	inline Vec2<T> max(const Vec2<T> &pa, const Vec2<T> &pb) { return Vec2<T>(glm::max(pa.x, pb.x), glm::max(pa.y, pb.y)); }

	template <typename T>
	inline T dot(const Vec3<T> &n1, const Vec3<T> &v2) { return glm::dot(n1, v2); }

	template <typename T>
	inline T absDot(const Vec3<T> &n1, const Vec3<T> &v2) { return glm::abs(glm::dot(n1, v2)); }

	template <typename T>
	inline Vec3<T> cross(const Vec3<T> &v1, const Vec3<T> &v2) { return glm::cross(v1, v2); }

	template <typename T>
	inline Vec3<T> normalize(const Vec3<T> &v) { return glm::normalize(v); }

	template <typename T>
	inline T minComponent(const Vec3<T> &v) { return glm::min(v.x, glm::min(v.y, v.z)); }

	template <typename T>
	inline T maxComponent(const Vec3<T> &v) { return glm::max(v.x, glm::max(v.y, v.z)); }

	template <typename T>
	inline int maxDimension(const Vec3<T> &v) { return (v.x > v.y) ? ((v.x > v.z) ? 0 : 2) : ((v.y > v.z) ? 1 : 2); }

	template <typename T>
	inline Vec3<T> permute(const Vec3<T> &v, int x, int y, int z) { return Vec3<T>(v[x], v[y], v[z]); }

	template <typename T>
	inline Vec3<T> faceforward(const Vec3<T> &n, const Vec3<T> &v) { return (dot(n, v) < 0.f) ? -n : n; }

	template <typename T>
	inline void coordinateSystem(const Vec3<T> &v1, Vec3<T> &v2, Vec3<T> &v3)
	{
		if (glm::abs(v1.x) > glm::abs(v1.y))
			v2 = Vec3<T>(-v1.z, 0, v1.x) / glm::sqrt(v1.x * v1.x + v1.z * v1.z);
		else
			v2 = Vec3<T>(0, v1.z, -v1.y) / glm::sqrt(v1.y * v1.y + v1.z * v1.z);
		v3 = cross(v1, v2);
	}

	inline Vec3f sphericalDirection(Float sinTheta, Float cosTheta, Float phi)
	{
		return Vec3f(sinTheta * glm::cos(phi), sinTheta * glm::sin(phi), cosTheta);
	}

	inline Vec3f sphericalDirection(Float sinTheta, Float cosTheta, Float phi,
		const Vec3f &x, const Vec3f &y, const Vec3f &z)
	{
		return sinTheta * glm::cos(phi) * x + sinTheta * glm::sin(phi) * y + cosTheta * z;
	}

	template <typename T>
	inline BBox3<T> unionBounds(const BBox3<T> &b, const Vec3<T> &p)
	{
		BBox3<T> ret;
		ret.m_pMin = min(b.m_pMin, p);
		ret.m_pMax = max(b.m_pMax, p);
		return ret;
	}

	template <typename T>
	inline BBox3<T> unionBounds(const BBox3<T> &b1, const BBox3<T> &b2)
	{
		BBox3<T> ret;
		ret.m_pMin = min(b1.m_pMin, b2.m_pMin);
		ret.m_pMax = max(b1.m_pMax, b2.m_pMax);
		return ret;
	}

	template <typename T>
	inline BBox3<T> intersect(const BBox3<T> &b1, const BBox3<T> &b2)
	{
		// Important: assign to pMin/pMax directly and don't run the BBox2()
		// constructor, since it takes min/max of the points passed to it.  In
		// turn, that breaks returning an invalid bound for the case where we
		// intersect non-overlapping bounds (as we'd like to happen).
		BBox3<T> ret;
		ret.m_pMin = max(b1.m_pMin, b2.m_pMin);
		ret.m_pMax = Min(b1.m_pMax, b2.m_pMax);
		return ret;
	}

	template <typename T>
	inline bool overlaps(const BBox3<T> &b1, const BBox3<T> &b2)
	{
		bool x = (b1.m_pMax.x >= b2.m_pMin.x) && (b1.m_pMin.x <= b2.m_pMax.x);
		bool y = (b1.m_pMax.y >= b2.m_pMin.y) && (b1.m_pMin.y <= b2.m_pMax.y);
		bool z = (b1.m_pMax.z >= b2.m_pMin.z) && (b1.m_pMin.z <= b2.m_pMax.z);
		return (x && y && z);
	}

	template <typename T>
	inline bool inside(const Vec3<T> &p, const BBox3<T> &b)
	{
		return (p.x >= b.m_pMin.x && p.x <= b.m_pMax.x && p.y >= b.m_pMin.y &&
			p.y <= b.m_pMax.y && p.z >= b.m_pMin.z && p.z <= b.m_pMax.z);
	}

	template <typename T>
	inline BBox2<T> unionBounds(const BBox2<T> &b, const Vec2<T> &p)
	{
		BBox2<T> ret;
		ret.m_pMin = min(b.m_pMin, p);
		ret.m_pMax = max(b.m_pMax, p);
		return ret;
	}

	template <typename T>
	inline BBox2<T> unionBounds(const BBox2<T> &b, const BBox2<T> &b2)
	{
		BBox2<T> ret;
		ret.m_pMin = min(b.m_pMin, b2.m_pMin);
		ret.m_pMax = max(b.m_pMax, b2.m_pMax);
		return ret;
	}

	template <typename T>
	inline BBox2<T> intersect(const BBox2<T> &b1, const BBox2<T> &b2)
	{
		// Important: assign to pMin/pMax directly and don't run the Bounds2()
		// constructor, since it takes min/max of the points passed to it.  In
		// turn, that breaks returning an invalid bound for the case where we
		// intersect non-overlapping bounds (as we'd like to happen).
		BBox2<T> ret;
		ret.m_pMin = max(b1.m_pMin, b2.m_pMin);
		ret.m_pMax = min(b1.m_pMax, b2.m_pMax);
		return ret;
	}

	template <typename T>
	inline bool overlaps(const BBox2<T> &ba, const BBox2<T> &bb)
	{
		bool x = (ba.m_pMax.x >= bb.m_pMin.x) && (ba.m_pMin.x <= bb.m_pMax.x);
		bool y = (ba.m_pMax.y >= bb.m_pMin.y) && (ba.m_pMin.y <= bb.m_pMax.y);
		return (x && y);
	}

	template <typename T>
	inline bool inside(const Vec2<T> &pt, const BBox2<T> &b)
	{
		return (pt.x >= b.m_pMin.x && pt.x <= b.m_pMax.x && pt.y >= b.m_pMin.y && pt.y <= b.m_pMax.y);
	}

	template <typename T>
	bool insideExclusive(const Vec2<T> &pt, const BBox2<T> &b) 
	{
		return (pt.x >= b.m_pMin.x && pt.x < b.m_pMax.x && pt.y >= b.m_pMin.y && pt.y < b.m_pMax.y);
	}

	template <typename T>
	inline bool BBox3<T>::hit(const Ray &ray, Float &hitt0, Float &hitt1) const
	{
		Float t0 = 0, t1 = ray.m_tMax;
		for (int i = 0; i < 3; ++i)
		{
			// Update interval for _i_th bounding box slab
			Float invRayDir = 1 / ray.m_dir[i];
			Float tNear = (m_pMin[i] - ray.m_origin[i]) * invRayDir;
			Float tFar = (m_pMax[i] - ray.m_origin[i]) * invRayDir;

			// Update parametric interval from slab intersection $t$ values
			if (tNear > tFar)
			{
				std::swap(tNear, tFar);
			}

			// Update _tFar_ to ensure robust ray--bounds intersection
			tFar *= 1 + 2 * gamma(3);
			t0 = tNear > t0 ? tNear : t0;
			t1 = tFar < t1 ? tFar : t1;
			if (t0 > t1)
			{
				return false;
			}
		}
		hitt0 = t0;
		hitt1 = t1;
		return true;
	}

	template <typename T>
	inline bool BBox3<T>::hit(const Ray &ray, const Vec3f &invDir, const int dirIsNeg[3]) const
	{
		const BBox3f &bounds = *this;
		// Check for ray intersection against $x$ and $y$ slabs
		Float tMin = (bounds[dirIsNeg[0]].x - ray.m_origin.x) * invDir.x;
		Float tMax = (bounds[1 - dirIsNeg[0]].x - ray.m_origin.x) * invDir.x;
		Float tyMin = (bounds[dirIsNeg[1]].y - ray.m_origin.y) * invDir.y;
		Float tyMax = (bounds[1 - dirIsNeg[1]].y - ray.m_origin.y) * invDir.y;

		// Update _tMax_ and _tyMax_ to ensure robust bounds intersection
		tMax *= 1 + 2 * gamma(3);
		tyMax *= 1 + 2 * gamma(3);
		if (tMin > tyMax || tyMin > tMax)
			return false;
		if (tyMin > tMin)
			tMin = tyMin;
		if (tyMax < tMax)
			tMax = tyMax;

		// Check for ray intersection against $z$ slab
		Float tzMin = (bounds[dirIsNeg[2]].z - ray.m_origin.z) * invDir.z;
		Float tzMax = (bounds[1 - dirIsNeg[2]].z - ray.om_origin.z) * invDir.z;

		// Update _tzMax_ to ensure robust bounds intersection
		tzMax *= 1 + 2 * gamma(3);
		if (tMin > tzMax || tzMin > tMax)
			return false;
		if (tzMin > tMin)
			tMin = tzMin;
		if (tzMax < tMax)
			tMax = tzMax;
		return (tMin < ray.m_tMax) && (tMax > 0);
	}

	template <typename Predicate>
	int findInterval(int size, const Predicate &pred) 
	{
		int first = 0, len = size;
		while (len > 0) 
		{
			int half = len >> 1, middle = first + half;
			// Bisect range based on value of _pred_ at _middle_
			if (pred(middle)) 
			{
				first = middle + 1;
				len -= half + 1;
			}
			else
			{
				len = half;
			}
		}
		return clamp(first - 1, 0, size - 2);
	}

	inline Vec3f reflect(const Vec3f &wo, const Vec3f &n) { return -wo + 2 * dot(wo, n) * n; }

	inline bool refract(const Vec3f &wi, const Vec3f &n, Float eta, Vec3f &wt)
	{
		// Compute $\cos \theta_\roman{t}$ using Snell's law
		Float cosThetaI = dot(n, wi);
		Float sin2ThetaI = glm::max(Float(0), Float(1 - cosThetaI * cosThetaI));
		Float sin2ThetaT = eta * eta * sin2ThetaI;

		// Handle total internal reflection for transmission
		if (sin2ThetaT >= 1) 
			return false;
		Float cosThetaT = glm::sqrt(1 - sin2ThetaT);
		wt = eta * -wi + (eta * cosThetaI - cosThetaT) * Vec3f(n);
		return true;
	}
}


namespace RT
{
	// Random Number Declarations
	static const double aDoubleOneMinusEpsilon = 0.99999999999999989;
	static const float aFloatOneMinusEpsilon = 0.99999994;

#ifdef AURORA_DOUBLE_AS_FLOAT
	static const Float aOneMinusEpsilon = aDoubleOneMinusEpsilon;
#else
	static const Float aOneMinusEpsilon = aFloatOneMinusEpsilon;
#endif

#define PCG32_DEFAULT_STATE 0x853c49e6748fea9bULL
#define PCG32_DEFAULT_STREAM 0xda3e39cb94b95bdbULL
#define PCG32_MULT 0x5851f42d4c957f2dULL

	class ARng
	{
	public:

		ARng();

		ARng(uint64_t sequenceIndex) { setSequence(sequenceIndex); }

		void setSequence(uint64_t sequenceIndex);

		uint32_t uniformUInt32();

		uint32_t uniformUInt32(uint32_t b)
		{
			uint32_t threshold = (~b + 1u) % b;
			while (true)
			{
				uint32_t r = uniformUInt32();
				if (r >= threshold)
					return r % b;
			}
		}

		Float uniformFloat()
		{
			//[0, 1)
			return glm::min(aOneMinusEpsilon, Float(uniformUInt32() * 2.3283064365386963e-10f));
		}

		template <typename Iterator>
		void shuffle(Iterator begin, Iterator end)
		{
			for (Iterator it = end - 1; it > begin; --it)
				std::iter_swap(it, begin + uniformUInt32((uint32_t)(it - begin + 1)));
		}

		void advance(int64_t idelta)
		{
			uint64_t cur_mult = PCG32_MULT, cur_plus = inc, acc_mult = 1u,
				acc_plus = 0u, delta = (uint64_t)idelta;
			while (delta > 0)
			{
				if (delta & 1)
				{
					acc_mult *= cur_mult;
					acc_plus = acc_plus * cur_mult + cur_plus;
				}
				cur_plus = (cur_mult + 1) * cur_plus;
				cur_mult *= cur_mult;
				delta /= 2;
			}
			state = acc_mult * state + acc_plus;
		}

		int64_t operator-(const ARng& other) const
		{
			CHECK_EQ(inc, other.inc);
			uint64_t cur_mult = PCG32_MULT, cur_plus = inc, cur_state = other.state,
				the_bit = 1u, distance = 0u;
			while (state != cur_state)
			{
				if ((state & the_bit) != (cur_state & the_bit))
				{
					cur_state = cur_state * cur_mult + cur_plus;
					distance |= the_bit;
				}
				CHECK_EQ(state & the_bit, cur_state & the_bit);
				the_bit <<= 1;
				cur_plus = (cur_mult + 1ULL) * cur_plus;
				cur_mult *= cur_mult;
			}
			return (int64_t)distance;
		}

	private:
		uint64_t state, inc;
	};

	inline ARng::ARng() : state(PCG32_DEFAULT_STATE), inc(PCG32_DEFAULT_STREAM) {}

	inline void ARng::setSequence(uint64_t initseq)
	{
		state = 0u;
		inc = (initseq << 1u) | 1u;
		uniformUInt32();
		state += PCG32_DEFAULT_STATE;
		uniformUInt32();
	}

	inline uint32_t ARng::uniformUInt32()
	{
		uint64_t oldstate = state;
		state = oldstate * PCG32_MULT + inc;
		uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
		uint32_t rot = (uint32_t)(oldstate >> 59u);
		return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
	}
}