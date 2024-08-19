
#include <iostream>

#include "Scene/Scene.h"
#include "Render/Render.h"
#include "Scene/SceneParser.h"

using namespace std;
using namespace RT;



int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);

	std::cout << "输入场景地址" << std::endl;
	std::string filename;
	std::cin >> filename;


	Scene::ptr scene = nullptr;
	Renderer::ptr Renderer = nullptr;

	//加载场景
	Parser::parser(filename, scene, Renderer);

	CHECK_NE(scene, nullptr);
	CHECK_NE(Renderer, nullptr);

	Renderer->preprocess(*scene);
	Renderer->render(*scene);
	

	return 0;
}