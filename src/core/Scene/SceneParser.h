#ifndef ARPARSER_H
#define ARPARSER_H

#include "Utils/Base.h"
#include "Utils/Math.h"
#include "Render/Render.h"
#include "Scene/Scene.h"

#include "nlohmann/json.hpp"

namespace RT
{
	class Parser final
	{
	public:

		static void parser(const std::string &path, Scene::ptr &_scene, Renderer::ptr &Renderer);

	private:
		using json_value_type = nlohmann::basic_json<>::value_type;

	};
}

#endif