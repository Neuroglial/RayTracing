#pragma once

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Object/Object.h"
#include "Object/Hitable.h"
#include "Shape/TriangleShape.h"

namespace RT
{
	//! @brief Basic unit of objects in world.
	/**
	 * An entity is the very basic concept in a world. Everything, including camera, mesh, light or anything else is an
	 * entity. An entity could parse itself and decouple itself into one or multiple primitives depending its complexity.
	 * An entity itself doesn't touch rendering directly. It serves as a place where the logic operations should be performed.
	 */
	class Entity : public Object
	{
	public:
		typedef std::shared_ptr<Entity> ptr;

		Entity() = default;
		Entity(const PropertyTreeNode &node);

		Material* getMaterial() const { return m_material.get(); }
		const std::vector<Hitable::ptr>& getHitables() const { return m_hitables; }

		virtual std::string toString() const override { return "Entity[]"; }
		virtual ClassType getClassType() const override { return ClassType::AEHitable; }

	protected:
		Material::ptr m_material;
		std::vector<Hitable::ptr> m_hitables;
		Transform m_objectToWorld, m_worldToObject;

	};

	class MeshEntity : public Entity
	{
	public:
		typedef std::shared_ptr<MeshEntity> ptr;

		MeshEntity(const PropertyTreeNode &node);

		virtual std::string toString() const override { return "MeshEntity[]"; }

	private:
		TriangleMesh::unique_ptr m_mesh;
	};

}