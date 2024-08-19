#ifndef ARTRIANGLE_SHAPE_H
#define ARTRIANGLE_SHAPE_H

#include "Shape/Shape.h"

namespace RT
{
	class TriangleMesh final
	{
	public:
		typedef std::shared_ptr<TriangleMesh> ptr;
		typedef std::unique_ptr<TriangleMesh> unique_ptr;

		TriangleMesh(Transform *objectToWorld, const std::string &filename);

		size_t numTriangles() const { return m_indices.size() / 3; }
		size_t numVertices() const { return m_nVertices; }

		bool hasUV() const { return m_uv != nullptr; }
		bool hasNormal() const { return m_normal != nullptr; }

		const Vec3f& getPosition(const int &index) const { return m_position[index]; }
		const Vec3f& getNormal(const int &index) const { return m_normal[index]; }
		const Vec2f& getUV(const int &index) const { return m_uv[index]; }

		const std::vector<int>& getIndices() const { return m_indices; }

	private:

		// TriangleMesh Data
		std::unique_ptr<Vec3f[]> m_position = nullptr;
		std::unique_ptr<Vec3f[]> m_normal = nullptr;
		std::unique_ptr<Vec2f[]> m_uv = nullptr;
		std::vector<int> m_indices;
		int m_nVertices;
	};

	class ATriangleShape final : public Shape
	{
	public:
		typedef std::shared_ptr<ATriangleShape> ptr;

		ATriangleShape(const PropertyTreeNode &node);
		ATriangleShape(Transform *objectToWorld, Transform *worldToObject,
			std::array<int, 3> indices, TriangleMesh *mesh);

		virtual ~ATriangleShape() = default;

		virtual Float area() const override;

		virtual Interaction sample(const Vec2f &u, Float &pdf) const override;

		virtual BBox3f objectBound() const override;
		virtual BBox3f worldBound() const override;

		virtual bool hit(const Ray &ray) const override;
		virtual bool hit(const Ray &ray, Float &tHit, SurfaceInteraction &isect) const override;

		virtual Float solidAngle(const Vec3f &p, int nSamples = 512) const override;

		virtual std::string toString() const override { return "TriangleShape[]"; }

	private:
		TriangleMesh *m_mesh;
		std::array<int, 3> m_indices;
	};
}

#endif