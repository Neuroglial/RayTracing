#include "Scene/SceneParser.h"

#include <fstream>

#include "Object/Film.h"
#include "Render/Sampler.h"
#include "Render/Filter.h"
#include "Shape/Shape.h"
#include "Camera/Camera.h"
#include "Material/Material.h"
#include "Render/Light.h"
#include "Accelerators/KDTree.h"
//#include "accelerators/LinearAggregate.h"

using namespace nlohmann;

namespace RT
{
	//加载场景
	void Parser::parser(
		const std::string &path,
		Scene::ptr &_scene,
		Renderer::ptr &_renderer)
	{
		_renderer = nullptr;
		_scene = nullptr;
		
		json _scene_json;

		{
			std::ifstream infile(path);

			if (!infile)
			{
				LOG(ERROR) << "Could not open the json file: " << path;
			}

			infile >> _scene_json;
			infile.close();
		}
		LOG(INFO) << "Parse the scene file from " << path;

		//获取该句柄类型
		auto get_func = [](const json_value_type &target) -> std::string
		{
			if (target.is_string())
			{
				return target.get<std::string>();
			}
			else if (target.is_boolean())
			{
				bool ret = target.get<bool>();
				return ret ? "true" : "false";
			}
			else if (target.is_number_float())
			{
				float ret = target.get<float>();
				std::stringstream ss;
				ss << ret;
				return ss.str();
			}
			else
			{
				int ret = target.get<int>();
				std::stringstream ss;
				ss << ret;
				return ss.str();
			}
		};

		//写入根目录
		{
			size_t last_slash_idx = path.rfind('\\');
			if (last_slash_idx == std::string::npos)
			{
				last_slash_idx = path.rfind('/');
			}
			if (last_slash_idx != std::string::npos)
			{
				PropertyTreeNode::m_directory = path.substr(0, last_slash_idx + 1);
			}
		}

		//建立资产节点函数
		std::function<PropertyTreeNode(const std::string &tag, const json_value_type &jsonData)> build_tree_func;
		build_tree_func = [&](const std::string &tag, const json_value_type &jsonData) -> PropertyTreeNode
		{	
			PropertyTreeNode node(tag);
			if (jsonData.is_object())
			{
				for (const auto& item : jsonData.items())
				{
					const auto &key = item.key();
					const auto &value = item.value();
					if (!value.is_object())
					{
						std::vector<std::string> values;
						if (value.is_array())
						{
							for (int i = 0; i < value.size(); ++i)
							{
								values.push_back(get_func(value[i]));
							}
						}
						else
						{
							values.push_back(get_func(value));
						}
						node.addProperty(key, values);
					}
					else
					{
						//创建子节点
						node.addChild(build_tree_func(key, jsonData[key]));
					}

				}
			}
			return node;
		};

		//加载渲染器
		{
			if (!_scene_json.contains("Renderer"))
			{
				LOG(ERROR) << "There is no renderer in " << path;
			}
			PropertyTreeNode rendererNode = build_tree_func("Renderer", _scene_json["Renderer"]);
			_renderer = Renderer::ptr(static_cast<Renderer*>(ObjectFactory::createInstance(
				rendererNode.getTypeName(), rendererNode)));
		}

		std::vector<Light::ptr> _lights;
		std::vector<Entity::ptr> _entities;
		std::vector<Hitable::ptr> _hitables;

		//加载实体
		{
			if (!_scene_json.contains("Entity"))
			{
				LOG(ERROR) << "There is no Entity in " << path;
			}
			const auto &entities_json = _scene_json["Entity"];
			for (int i = 0; i < entities_json.size(); ++i)
			{
				//获取实体的参数
				PropertyTreeNode entityNode = build_tree_func("Entity", entities_json[i]);

				//构造实体
				Entity::ptr entity = Entity::ptr(static_cast<Entity*>(ObjectFactory::createInstance(
					entityNode.getTypeName(), entityNode)));
				_entities.push_back(entity);
			}

			//提取实体中的可碰撞
			for (auto &entity : _entities)
			{
				for (const auto &hitable : entity->getHitables())
				{
					_hitables.push_back(hitable);
				}
			}



			for (int i = 0; i < _hitables.size(); ++i)
			{
				if (_hitables[i]->getAreaLight() != nullptr)
				{
					_lights.push_back(Light::ptr(dynamic_cast<HitableObject*>(_hitables[i].get())->getAreaLightPtr()));
				}
			}
		}

		//生成KD树
		KdTree::ptr _aggregate = std::make_shared<KdTree>(_hitables);
		_scene = std::make_shared<Scene>(_entities, _aggregate, _lights);

	}

}