#include "Object/Object.h"

namespace RT
{
	//----------------------------------------------------APropertyList-----------------------------------------------------

	void PropertyList::set(const std::string &name, const std::string &value)
	{
		if (m_properties.find(name) != m_properties.end())
			LOG(ERROR) << "Property \"" << name << "\" was specified multiple times!";
		auto &prop = m_properties[name];
		prop.addValue(value);
	}

	void PropertyList::set(const std::string &name, const std::vector<std::string> &values)
	{
		if (m_properties.find(name) != m_properties.end())
			LOG(ERROR) << "Property \"" << name << "\" was specified multiple times!";
		CHECK_GT(values.size(), 0);
		auto &prop = m_properties[name];
		prop.setValue(values);
	}

	bool PropertyList::has(const std::string &name) const { return m_properties.find(name) != m_properties.end(); }

	bool PropertyList::getBoolean(const std::string &name) const { return get<bool>(name, 0); }
	bool PropertyList::getBoolean(const std::string &name, const bool &defaultValue) const { return get<bool>(name, 0, defaultValue); }

	Float PropertyList::getFloat(const std::string &name) const { return get<Float>(name, 0); }
	Float PropertyList::getFloat(const std::string &name, const Float &defaultValue) const { return get<Float>(name, 0, defaultValue); }

	int PropertyList::getInteger(const std::string &name) const { return get<Float>(name, 0); }
	int PropertyList::getInteger(const std::string &name, const int &defaultValue) const { return get<Float>(name, 0, defaultValue); }

	std::string PropertyList::getString(const std::string &name) const { return get<std::string>(name, 0); }
	std::string PropertyList::getString(const std::string &name, const std::string &defaultValue) const { return get<std::string>(name, 0, defaultValue); }

	Vec2f PropertyList::getVector2f(const std::string &name) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			LOG(ERROR) << "Property \"" << name << "\" is missing!";

		if (it->second.size() < 2)
			LOG(ERROR) << "Property \"" << name << "\" doesn't have 2 components!";

		std::array<Float, 2> values = getVector<2>(it->second);
		return Vec2f(values[0], values[1]);
	}

	Vec2f PropertyList::getVector2f(const std::string &name, const Vec2f &defaultValue) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			return defaultValue;

		if (it->second.size() < 2)
			LOG(ERROR) << "Property \"" << name << "\" doesn't have 2 components!";

		std::array<Float, 2> values = getVector<2>(it->second);
		return Vec2f(values[0], values[1]);
	}

	Vec3f PropertyList::getVector3f(const std::string &name) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			LOG(ERROR) << "Property \"" << name << "\" is missing!";
		
		if (it->second.size() < 3)
			LOG(ERROR) << "Property \"" << name << "\" doesn't have 3 components!";

		std::array<Float, 3> values = getVector<3>(it->second);
		return Vec3f(values[0], values[1], values[2]);
	}

	Vec3f PropertyList::getVector3f(const std::string &name, const Vec3f &defaultValue) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			return defaultValue;

		if (it->second.size() < 3)
			LOG(ERROR) << "Property \"" << name << "\" doesn't have 3 components!";

		std::array<Float, 3> values = getVector<3>(it->second);
		return Vec3f(values[0], values[1], values[2]);
	}

	std::vector<Float> PropertyList::getVectorNf(const std::string &name) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			LOG(ERROR) << "Property \"" << name << "\" is missing!";

		const auto &prop = it->second;

		std::vector<Float> values(prop.size());
		auto get_func = [&](const size_t &index) -> Float
		{
			auto valueStr = prop[index];
			std::istringstream iss(valueStr);
			Float value;
			iss >> value;
			return  value;
		};

		for (size_t i = 0; i < prop.size(); ++i)
			values[i] = get_func(i);

		return values;
	}

	std::vector<Float> PropertyList::getVectorNf(const std::string &name, const std::vector<Float> &defaultValue) const
	{
		auto it = m_properties.find(name);
		if (it == m_properties.end())
			return defaultValue;

		const auto &prop = it->second;

		std::vector<Float> values(prop.size());
		auto get_func = [&](const size_t &index) -> Float
		{
			auto valueStr = prop[index];
			std::istringstream iss(valueStr);
			Float value;
			iss >> value;
			return  value;
		};

		for (size_t i = 0; i < prop.size(); ++i)
			values[i] = get_func(i);

		return values;
	}

	//----------------------------------------------------APropertyTreeNode-----------------------------------------------------

	std::string PropertyTreeNode::m_directory = "";

	std::string PropertyTreeNode::getTypeName() const 
	{ 
		return m_property.getString("Type"); 
	}

	const PropertyList& PropertyTreeNode::getPropertyList() const { return m_property; }

	bool PropertyTreeNode::hasProperty(const std::string &name) const { return m_property.has(name); }

	bool PropertyTreeNode::hasPropertyChild(const std::string &name) const
	{
		for (int i = 0; i < m_children.size(); ++i)
		{
			if (m_children[i].getNodeName() == name)
			{
				return true;
			}
		}
		return false;
	}

	const PropertyTreeNode& PropertyTreeNode::getPropertyChild(const std::string &name) const
	{
		for (int i = 0; i < m_children.size(); ++i)
		{
			if (m_children[i].getNodeName() == name)
			{
				return m_children[i];
			}
		}
		LOG(ERROR) << "No property for " << name;
	}

	void PropertyTreeNode::addProperty(const std::string &name, const std::string &value) 
	{ 
		m_property.set(name, value); 
	}

	void PropertyTreeNode::addProperty(const std::string &name, const std::vector<std::string> &values) 
	{ 
		m_property.set(name, values); 
	}

	void PropertyTreeNode::addChild(const PropertyTreeNode &child) { m_children.push_back(child); }

	//---------------------------------------------------------AObject----------------------------------------------------------

	void Object::activate() { /* Do nothing */ }

	void Object::addChild(Object *child)
	{
		LOG(FATAL) << "NoriObject::addChild() is not implemented for objects of type "
			<< getClassTypeName(getClassType()) << "!";
	}

	void Object::setParent(Object *) { /* Do nothing */ }

	//-------------------------------------------AObjectFactory-------------------------------------

	std::map<std::string, ObjectFactory::Constructor> &ObjectFactory::getConstrMap()
	{
		static std::map<std::string, Constructor> constrMap;
		return constrMap;
	}

	void ObjectFactory::registerClass(const std::string &type, const Constructor &constr)
	{
		auto &constrMap = getConstrMap();
		constrMap.insert({ type, constr });
	}

	Object *ObjectFactory::createInstance(const std::string &type, const PropertyTreeNode &node)
	{
		auto &constrMap = getConstrMap();
		if (constrMap.find(type) == constrMap.end())
			LOG(ERROR) << "A constructor for class \"" << type << "\" could not be found!";
		return constrMap[type](node);
	}

}