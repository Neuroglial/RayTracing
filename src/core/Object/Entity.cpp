#include "Object/Entity.h"

#include "Shape/Shape.h"

namespace RT
{
	//-------------------------------------------Entity-------------------------------------

	AURORA_REGISTER_CLASS(Entity, "Entity")

	Entity::Entity(const PropertyTreeNode &node)
	{
		const PropertyList& props = node.getPropertyList();

		// 形状
		const auto &shapeNode = node.getPropertyChild("Shape");
		Shape::ptr shape = Shape::ptr(static_cast<Shape*>(ObjectFactory::createInstance(
			shapeNode.getTypeName(), shapeNode)));
		shape->setTransform(&m_objectToWorld, &m_worldToObject);

		// 变换
		Transform objectToWrold;
		const auto &shapeProps = shapeNode.getPropertyList();
		if (shapeNode.hasProperty("Transform"))
		{
			std::vector<Transform> transformStack;
			std::vector<Float> sequence = shapeProps.getVectorNf("Transform");
			size_t it = 0;
			bool undefined = false;
			while (it < sequence.size() && !undefined)
			{
				int token = static_cast<int>(sequence[it]);
				switch (token)
				{
				case 0://translate
					CHECK_LT(it + 3, sequence.size());
					Vec3f _trans = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(translate(_trans));
					it += 4;
					break;
				case 1://scale
					CHECK_LT(it + 3, sequence.size());
					Vec3f _scale = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(scale(_scale.x, _scale.y, _scale.z));
					it += 4;
					break;
				case 2://rotate
					CHECK_LT(it + 4, sequence.size());
					Vec3f axis = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(rotate(sequence[it + 4], axis));
					it += 5;
					break;
				default:
					undefined = true;
					LOG(ERROR) << "Undefined transform action";
					break;
				}
			}

			//Note: calculate the transform matrix in a first-in-last-out manner
			if (!undefined)
			{
				for (auto it = transformStack.rbegin(); it != transformStack.rend(); ++it)
				{
					objectToWrold = objectToWrold * (*it);
				}
			}
		}
		m_objectToWorld = objectToWrold;
		m_worldToObject = inverse(m_objectToWorld);

		// 材质
		const auto &materialNode = node.getPropertyChild("Material");
		m_material = Material::ptr(static_cast<Material*>(ObjectFactory::createInstance(
				materialNode.getTypeName(), materialNode)));

		//面光
		AreaLight::ptr areaLight = nullptr;
		if (node.hasPropertyChild("Light"))
		{
			const auto &lightNode = node.getPropertyChild("Light");
			areaLight = AreaLight::ptr(static_cast<AreaLight*>(ObjectFactory::createInstance(
				lightNode.getTypeName(), lightNode)));
		}

		m_hitables.push_back(std::make_shared<HitableObject>(shape, m_material.get(), areaLight));
	}

	//-------------------------------------------AMeshEntity-------------------------------------

	AURORA_REGISTER_CLASS(MeshEntity, "MeshEntity")

	MeshEntity::MeshEntity(const PropertyTreeNode &node)
	{
		const PropertyList& props = node.getPropertyList();
		const std::string filename = props.getString("Filename");

		// 形状
		const auto &shapeNode = node.getPropertyChild("Shape");

		// 变换
		Transform objectToWrold;
		const auto &shapeProps = shapeNode.getPropertyList();
		if (shapeNode.hasProperty("Transform"))
		{
			std::vector<Transform> transformStack;
			std::vector<Float> sequence = shapeProps.getVectorNf("Transform");
			size_t it = 0;
			bool undefined = false;
			while (it < sequence.size() && !undefined)
			{
				int token = static_cast<int>(sequence[it]);
				switch (token)
				{
				case 0://translate
					CHECK_LT(it + 3, sequence.size());
					Vec3f _trans = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(translate(_trans));
					it += 4;
					break;
				case 1://scale
					CHECK_LT(it + 3, sequence.size());
					Vec3f _scale = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(scale(_scale.x, _scale.y, _scale.z));
					it += 4;
					break;
				case 2://rotate
					CHECK_LT(it + 4, sequence.size());
					Vec3f axis = Vec3f(sequence[it + 1], sequence[it + 2], sequence[it + 3]);
					transformStack.push_back(rotate(sequence[it + 4], axis));
					it += 5;
					break;
				default:
					undefined = true;
					LOG(ERROR) << "Undefined transform action";
					break;
				}
			}

			//Note: calculate the transform matrix in a first-in-last-out manner
			if (!undefined)
			{
				for (auto it = transformStack.rbegin(); it != transformStack.rend(); ++it)
				{
					objectToWrold = objectToWrold * (*it);
				}
			}
		}
		m_objectToWorld = objectToWrold;
		m_worldToObject = inverse(m_objectToWorld);

		//材质
		const auto &materialNode = node.getPropertyChild("Material");
		m_material = Material::ptr(static_cast<Material*>(ObjectFactory::createInstance(
			materialNode.getTypeName(), materialNode)));

		//加载三角形
		m_mesh = TriangleMesh::unique_ptr(new TriangleMesh(&m_objectToWorld, PropertyTreeNode::m_directory + filename));
		const auto &meshIndices = m_mesh->getIndices();
		for (size_t i = 0; i < meshIndices.size(); i += 3)
		{
			std::array<int, 3> indices;
			indices[0] = meshIndices[i + 0];
			indices[1] = meshIndices[i + 1];
			indices[2] = meshIndices[i + 2];
			ATriangleShape::ptr triangle = std::make_shared<ATriangleShape>(&m_objectToWorld, &m_worldToObject, indices, m_mesh.get());

			//是否为面光
			AreaLight::ptr areaLight = nullptr;
			if (node.hasPropertyChild("Light"))
			{
				const auto &lightNode = node.getPropertyChild("Light");
				areaLight = AreaLight::ptr(static_cast<AreaLight*>(ObjectFactory::createInstance(
					lightNode.getTypeName(), lightNode)));
			}
			m_hitables.push_back(std::make_shared<HitableObject>(triangle, m_material.get(), areaLight));
		}
	}

}