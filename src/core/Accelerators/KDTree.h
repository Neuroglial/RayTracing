#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Object/Hitable.h"

namespace RT
{
	class KdTreeNode;
	class BoundEdge;
	class KdTree : public HitableAggregate
	{
	public:
		typedef std::shared_ptr<KdTree> ptr;

		KdTree(const std::vector<Hitable::ptr> &hitables, int isectCost = 80, int traversalCost = 1,
			Float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);

		virtual BBox3f worldBound() const override { return m_bounds; }
		~KdTree();

		virtual bool hit(const Ray &ray) const override;
		virtual bool hit(const Ray &ray, SurfaceInteraction &iset) const override;

		virtual std::string toString() const override { return "KdTree[]"; }

	private:

		void buildTree(int nodeNum, const BBox3f &bounds,
			const std::vector<BBox3f> &primBounds, int *primNums,
			int nprims, int depth,
			const std::unique_ptr<BoundEdge[]> edges[3], int *prims0,
			int *prims1, int badRefines = 0);
		
		// SAH split measurement
		const Float m_emptyBonus;
		const int m_isectCost, m_traversalCost, m_maxHitables;

		// Compact the node into an array
		KdTreeNode *m_nodes;
		int m_nAllocedNodes, m_nextFreeNode;
		
		BBox3f m_bounds;
		std::vector<Hitable::ptr> m_hitables;
		std::vector<int> m_hitableIndices;
	};

	struct KdToDo 
	{
		const KdTreeNode *node;
		Float tMin, tMax;
	};
}