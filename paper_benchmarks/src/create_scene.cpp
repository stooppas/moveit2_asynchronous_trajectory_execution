#include "paper_benchmarks/scene.hpp"

rclcpp::Node::SharedPtr node;
std::shared_ptr<Scene> scene;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("create_scene");

  scene = std::make_shared<Scene>(node);

  scene->create_random_scene();

  rclcpp::shutdown();
}
