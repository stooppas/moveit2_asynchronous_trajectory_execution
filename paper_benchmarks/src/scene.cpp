#include "paper_benchmarks/scene.hpp"


Scene::Scene(rclcpp::Node::SharedPtr node){

  this->node = node;
  planning_scene_diff_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
  populate_blockList();
  create_scene();

}

void Scene::populate_blockList(){
  
}

void Scene::create_scene(){

  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  int counter = 1;

  for(auto block: blockList){
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "world";
    attached_object.object.header.frame_id = "world";
    attached_object.object.id = "block_" + std::to_string(counter++);
    block.id = attached_object.object.id;

    attached_object.object.primitives.push_back(block.primitive);
    attached_object.object.primitive_poses.push_back(block.pose);
    planning_scene.world.collision_objects.push_back(attached_object.object);

  }
  planning_scene_diff_publisher->publish(planning_scene);
}

bool Scene::attachObject(){
  return true;
}
bool Scene::detachObject(){
  return true;
}
