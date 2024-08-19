#include "Accelerators/KDTree.h"

#include "Utils/Memory.h"

namespace RT
{
	class KdTreeNode 
	{
	public:

		void initLeafNode(int *hitableIndices, int np, std::vector<int> *primitiveIndices);

		void initInteriorNode(int axis, int ac, Float split) 
		{
			// 只存储一个子指针的方式布局节点：所有节点都分配在一个连续的内存块中，负责分割平面下方空间的内部节点的子节点总是存储在其父节点之后的数组位置
			m_split = split;
			m_flags = axis;

			// 另一个子对象表示分割平面上方的空间，将最终到达数组中的其他位置，m_rightChildIndex将其位置存储在nodes数组中
			m_rightChildIndex |= (ac << 2);
		}

		Float splitPos() const { return m_split; }
		int numHitables() const { return m_nHitables >> 2; }
		int splitAxis() const { return m_flags & 3; }
		bool isLeaf() const { return (m_flags & 3) == 3; }
		int aboveChild() const { return m_rightChildIndex >> 2; }

		union 
		{
			Float m_split;                 // 分割位置
			int m_oneHitable;              // 叶子节点
			int m_hitableIndicesOffset;    // 叶子节点
		};

	private:

		union 
		{
			int m_rightChildIndex;// Interior
			int m_flags;		  // Both
			int m_nHitables;	  // 叶子节点
		};
	};

	void KdTreeNode::initLeafNode(int *hitableIndices, int np, std::vector<int> *primitiveIndices)
	{
		// m_flags的低位2位表示它是一个叶子节点，而m_nPrims的高位30位可用于记录有多少可碰撞项与其重叠
		m_flags = 3;
		m_nHitables |= (np << 2);

		if (np == 0)
		{
			m_oneHitable = 0;
		}
		else if (np == 1)
		{
			m_oneHitable = hitableIndices[0];
		}
		else 
		{
			// 如果有多个图元重叠，则它们的索引将存储在m_primitiveIndicesOffset的一个段中。叶子的第一个索引的偏移量存储在m_primitiveIndicesOffset中
			m_hitableIndicesOffset = primitiveIndices->size();
			for (int i = 0; i < np; ++i)
			{
				primitiveIndices->push_back(hitableIndices[i]);
			}
		}
	}

	enum class EdgeType { Start, End };
	class BoundEdge 
	{
	public:
		BoundEdge() = default;
		BoundEdge(Float t, int hitableIndex, bool starting) : m_t(t), m_hitableIndex(hitableIndex)
		{
			m_type = starting ? EdgeType::Start : EdgeType::End;
		}

		Float m_t;
		int m_hitableIndex;
		EdgeType m_type;
	};

	KdTree::KdTree(const std::vector<Hitable::ptr> &hitables, int isectCost/* = 80*/, int traversalCost/* = 1*/,
		Float emptyBonus/* = 0.5*/, int maxHitables/* = 1*/, int maxDepth/* = -1*/) : 
		m_isectCost(isectCost),
		m_traversalCost(traversalCost),
		m_maxHitables(maxHitables),
		m_emptyBonus(emptyBonus),
		m_hitables(hitables)
	{
		m_nextFreeNode = m_nAllocedNodes = 0;
		if (maxDepth <= 0)
		{
			maxDepth = std::round(8 + 1.3f * glm::log2(float(int64_t(m_hitables.size()))));
		}

		// 计算BoundingBox
		std::vector<BBox3f> hitableBounds;
		hitableBounds.reserve(m_hitables.size());
		for (const Hitable::ptr &hitable : m_hitables) 
		{
			BBox3f b = hitable->worldBound();
			m_bounds = unionBounds(m_bounds, b);
			hitableBounds.push_back(b);
		}

		// 申请内存
		std::unique_ptr<BoundEdge[]> edges[3];
		for (int i = 0; i < 3; ++i)
		{
			edges[i].reset(new BoundEdge[2 * m_hitables.size()]);
		}
		std::unique_ptr<int[]> leftNodeRoom(new int[m_hitables.size()]);
		std::unique_ptr<int[]> rightNodeRoom(new int[(maxDepth + 1) * m_hitables.size()]);

		// 初始化索引
		std::unique_ptr<int[]> hitableIndices(new int[m_hitables.size()]);
		for (size_t i = 0; i < m_hitables.size(); ++i)
		{
			hitableIndices[i] = i;
		}

		// 开始建立KD树
		buildTree(0, m_bounds, hitableBounds, hitableIndices.get(), m_hitables.size(),
			maxDepth, edges, leftNodeRoom.get(), rightNodeRoom.get());
	}

	void KdTree::buildTree(int nodeIndex, const BBox3f &nodeBounds,
		const std::vector<BBox3f> &allHitableBounds,
		int *hitableIndices, int nHitables, int depth,
		const std::unique_ptr<BoundEdge[]> edges[3],
		int *leftNodeRoom, int *rightNodeRoom, int badRefines)
	{
		CHECK_EQ(nodeIndex, m_nextFreeNode);

		// 获取下一个未使用节点
		if (m_nextFreeNode == m_nAllocedNodes)
		{
			int nNewAllocNodes = glm::max(2 * m_nAllocedNodes, 512);
			KdTreeNode *n = AllocAligned<KdTreeNode>(nNewAllocNodes);
			if (m_nAllocedNodes > 0) 
			{
				memcpy(n, m_nodes, m_nAllocedNodes * sizeof(KdTreeNode));
				FreeAligned(m_nodes);
			}
			m_nodes = n;
			m_nAllocedNodes = nNewAllocNodes;
		}
		++m_nextFreeNode;

		// 如果满足终止条件，则初始化叶节点
		if (nHitables <= m_maxHitables || depth == 0) 
		{
			m_nodes[nodeIndex].initLeafNode(hitableIndices, nHitables, &m_hitableIndices);
			return;
		}

		// 初始化节点并继续递归
		
		// 为节点选择分割轴位置
		int bestAxis = -1, bestOffset = -1;
		Float bestCost = Infinity;
		Float oldCost = m_isectCost * Float(nHitables);

		// 当前节点表面积
		const Float invTotalSA = 1 / nodeBounds.surfaceArea();
		Vec3f diagonal = nodeBounds.m_pMax - nodeBounds.m_pMin;

		// 选择要沿哪个轴拆分
		int axis = nodeBounds.maximumExtent();
		int retries = 0;

	retrySplit:
		// 初始化该轴的的边
		for (int i = 0; i < nHitables; ++i)
		{
			int hi = hitableIndices[i];
			const BBox3f &bounds = allHitableBounds[hi];
			edges[axis][2 * i    ] = BoundEdge(bounds.m_pMin[axis], hi, true);
			edges[axis][2 * i + 1] = BoundEdge(bounds.m_pMax[axis], hi, false);
		}

		// 对边进行排序_
		std::sort(&edges[axis][0], &edges[axis][2 * nHitables],
			[](const BoundEdge &e0, const BoundEdge &e1) -> bool
		{
			if (e0.m_t == e1.m_t)
			{
				return (int)e0.m_type < (int)e1.m_type;
			}
			else
			{
				return e0.m_t < e1.m_t;
			}
		});

		// 计算axis的所有拆分成本，以找到最佳结果
		int nBelow = 0, nAbove = nHitables;
		const auto &currentEdge = edges[axis];
		for (int i = 0; i < 2 * nHitables; ++i)
		{
			if (currentEdge[i].m_type == EdgeType::End)
				--nAbove;
			Float edgeT = currentEdge[i].m_t;
			if (edgeT > nodeBounds.m_pMin[axis] && edgeT < nodeBounds.m_pMax[axis])
			{
				// 计算在该点的拆分成本

				// 计算在edgeT处进行分割的子曲面面积
				int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
				Float belowSA = 2 * (diagonal[otherAxis0] * diagonal[otherAxis1] + (edgeT - nodeBounds.m_pMin[axis]) *
					(diagonal[otherAxis0] + diagonal[otherAxis1]));
				Float aboveSA = 2 * (diagonal[otherAxis0] * diagonal[otherAxis1] + (nodeBounds.m_pMax[axis] - edgeT) *
					(diagonal[otherAxis0] + diagonal[otherAxis1]));
				Float pBelow = belowSA * invTotalSA;
				Float pAbove = aboveSA * invTotalSA;
				Float eb = (nAbove == 0 || nBelow == 0) ? m_emptyBonus : 0;
				Float cost = m_traversalCost + m_isectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

				// 如果这是迄今为止成本最低的，则更新最佳分割
				if (cost < bestCost)
				{
					bestCost = cost;
					bestAxis = axis;
					bestOffset = i;
				}
			}
			if (currentEdge[i].m_type == EdgeType::Start)
				++nBelow;
		}
		CHECK(nBelow == nHitables && nAbove == 0);

		if (bestAxis == -1 && retries < 2) 
		{
			++retries;
			axis = (axis + 1) % 3;
			goto retrySplit;
		}

		// 如果没有找到好的分割位置，则创建叶子
		if (bestCost > oldCost) 
			++badRefines;
		if ((bestCost > 4 * oldCost && nHitables < 16) || bestAxis == -1 || badRefines == 3) 
		{
			m_nodes[nodeIndex].initLeafNode(hitableIndices, nHitables, &m_hitableIndices);
			return;
		}

		// 根据分割对图元进行分类
		int lnHitables = 0, rnHitables = 0;
		for (int i = 0; i < bestOffset; ++i)
		{
			if (edges[bestAxis][i].m_type == EdgeType::Start)
				leftNodeRoom[lnHitables++] = edges[bestAxis][i].m_hitableIndex;
		}
		for (int i = bestOffset + 1; i < 2 * nHitables; ++i)
		{
			if (edges[bestAxis][i].m_type == EdgeType::End)
				rightNodeRoom[rnHitables++] = edges[bestAxis][i].m_hitableIndex;
		}

		// Recursively initialize children nodes
		Float tSplit = edges[bestAxis][bestOffset].m_t;
		BBox3f bounds0 = nodeBounds, bounds1 = nodeBounds;
		bounds0.m_pMax[bestAxis] = bounds1.m_pMin[bestAxis] = tSplit;

		// below subtree node
		buildTree(nodeIndex + 1, bounds0, allHitableBounds, leftNodeRoom, lnHitables, depth - 1, edges,
			leftNodeRoom, rightNodeRoom + nHitables, badRefines);
		int aboveChildIndex = m_nextFreeNode;

		m_nodes[nodeIndex].initInteriorNode(bestAxis, aboveChildIndex, tSplit);

		// above subtree node
		buildTree(aboveChildIndex, bounds1, allHitableBounds, rightNodeRoom, rnHitables, depth - 1, edges,
			leftNodeRoom, rightNodeRoom + nHitables, badRefines);
	}

	KdTree::~KdTree() { FreeAligned(m_nodes); }

	bool KdTree::hit(const Ray &ray) const
	{
		// Compute initial parametric range of ray inside kd-tree extent
		Float tMin, tMax;
		if (!m_bounds.hit(ray, tMin, tMax)) 
		{
			return false;
		}

		// Prepare to traverse kd-tree for ray
		Vec3f invDir(1 / ray.m_dir.x, 1 / ray.m_dir.y, 1 / ray.m_dir.z);
		constexpr int maxTodo = 64;
		KdToDo todo[maxTodo];
		int todoPos = 0;
		const KdTreeNode *currNode = &m_nodes[0];
		while (currNode != nullptr)
		{
			if (currNode->isLeaf()) 
			{
				// Check for shadow ray intersections inside leaf node
				int nHitables = currNode->numHitables();
				if (nHitables == 1)
				{
					const Hitable::ptr &p = m_hitables[currNode->m_oneHitable];
					if (p->hit(ray)) 
					{
						return true;
					}
				}
				else 
				{
					for (int i = 0; i < nHitables; ++i)
					{
						int hitableIndex = m_hitableIndices[currNode->m_hitableIndicesOffset + i];
						const Hitable::ptr &p = m_hitables[hitableIndex];
						if (p->hit(ray)) 
						{
							return true;
						}
					}
				}

				// Grab next node to process from todo list
				if (todoPos > 0) 
				{
					currNode = todo[--todoPos].node;
					tMin = todo[todoPos].tMin;
					tMax = todo[todoPos].tMax;
				}
				else
				{
					break;
				}
			}
			else 
			{
				// Process kd-tree interior node

				// Compute parametric distance along ray to split plane
				int axis = currNode->splitAxis();
				Float tPlane = (currNode->splitPos() - ray.m_origin[axis]) * invDir[axis];

				// Get node children pointers for ray
				const KdTreeNode *firstChild, *secondChild;
				int belowFirst = (ray.m_origin[axis] < currNode->splitPos()) ||
					(ray.m_origin[axis] == currNode->splitPos() && ray.m_dir[axis] <= 0);
				if (belowFirst) 
				{
					firstChild = currNode + 1;
					secondChild = &m_nodes[currNode->aboveChild()];
				}
				else 
				{
					firstChild = &m_nodes[currNode->aboveChild()];
					secondChild = currNode + 1;
				}

				// Advance to next child node, possibly enqueue other child
				if (tPlane > tMax || tPlane <= 0)
				{
					currNode = firstChild;
				}
				else if (tPlane < tMin)
				{
					currNode = secondChild;
				}
				else 
				{
					// Enqueue _secondChild_ in todo list
					todo[todoPos].node = secondChild;
					todo[todoPos].tMin = tPlane;
					todo[todoPos].tMax = tMax;
					++todoPos;
					currNode = firstChild;
					tMax = tPlane;
				}
			}
		}
		return false;
	}

	bool KdTree::hit(const Ray &ray, SurfaceInteraction &isect) const
	{
		// Compute initial parametric range of ray inside kd-tree extent
		Float tMin, tMax;
		if (!m_bounds.hit(ray, tMin, tMax)) 
		{
			return false;
		}

		// Prepare to traverse kd-tree for ray
		Vec3f invDir(1 / ray.m_dir.x, 1 / ray.m_dir.y, 1 / ray.m_dir.z);
		const int maxTodo = 64;
		KdToDo todo[maxTodo];
		int todoPos = 0;

		// Traverse kd-tree nodes in order for ray
		bool hit = false;
		const KdTreeNode *currNode = &m_nodes[0];
		while (currNode != nullptr)
		{
			// Bail out if we found a hit closer than the current node
			if (ray.m_tMax < tMin) 
				break;

			// Process kd-tree interior node
			if (!currNode->isLeaf()) 
			{
				// Compute parametric distance along ray to split plane
				int axis = currNode->splitAxis();
				Float tPlane = (currNode->splitPos() - ray.m_origin[axis]) * invDir[axis];

				// Get node children pointers for ray
				const KdTreeNode *firstChild, *secondChild;
				int belowFirst = (ray.m_origin[axis] < currNode->splitPos()) ||
					(ray.m_origin[axis] == currNode->splitPos() && ray.m_dir[axis] <= 0);
				if (belowFirst) 
				{
					firstChild = currNode + 1;
					secondChild = &m_nodes[currNode->aboveChild()];
				}
				else 
				{
					firstChild = &m_nodes[currNode->aboveChild()];
					secondChild = currNode + 1;
				}

				// Advance to next child node, possibly enqueue other child
				if (tPlane > tMax || tPlane <= 0)
				{
					currNode = firstChild;
				}
				else if (tPlane < tMin)
				{
					currNode = secondChild;
				}
				else 
				{
					// Enqueue _secondChild_ in todo list
					todo[todoPos].node = secondChild;
					todo[todoPos].tMin = tPlane;
					todo[todoPos].tMax = tMax;
					++todoPos;
					currNode = firstChild;
					tMax = tPlane;
				}
			}
			else 
			{
				// Check for intersections inside leaf node
				int nHitables = currNode->numHitables();
				if (nHitables == 1)
				{
					const Hitable::ptr &p = m_hitables[currNode->m_oneHitable];
					// Check one hitable inside leaf node
					if (p->hit(ray, isect)) 
						hit = true;
				}
				else 
				{
					for (int i = 0; i < nHitables; ++i)
					{
						int index = m_hitableIndices[currNode->m_hitableIndicesOffset + i];
						const Hitable::ptr &p = m_hitables[index];
						// Check one hitable inside leaf node
						if (p->hit(ray, isect)) 
							hit = true;
					}
				}

				// Grab next node to process from todo list
				if (todoPos > 0) 
				{
					currNode = todo[--todoPos].node;
					tMin = todo[todoPos].tMin;
					tMax = todo[todoPos].tMax;
				}
				else
				{
					break;
				}
			}

		}

		return hit;
	}

}