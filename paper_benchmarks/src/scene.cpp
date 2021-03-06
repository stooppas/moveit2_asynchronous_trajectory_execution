#include "paper_benchmarks/scene.hpp"


Scene::Scene(rclcpp::Node::SharedPtr node){

  this->node = node;
  planning_scene_diff_publisher = node->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);

  while (planning_scene_diff_publisher->get_subscription_count() < 1)
  {
    rclcpp::sleep_for(std::chrono::milliseconds(500));
  }
}

void Scene::create_random_scene()
{
  moveit_msgs::msg::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  int counter = 1;
  float min_x = -0.45;
  float max_x = 0.45;
  float min_y = 0.35;
  float max_y = -0.35;
  while(counter < 30){
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "world";
      object.id = "box_" + std::to_string(counter);

      /* A default pose */
      geometry_msgs::msg::Pose pose;
      pose.position.x = min_x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x-min_x)));
      pose.position.y = min_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_y-min_y)));
      pose.position.z = 1.025;
      pose.orientation.z = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));;
      
      bool isColliding = false;

      for(auto object : planning_scene.world.collision_objects){
        double len = sqrt(pow(pose.position.x - object.primitive_poses[0].position.x,2) + pow(pose.position.y - object.primitive_poses[0].position.y,2));
        if(len < 0.08)
        {
          isColliding = true;
          break;
        }
      }

      if(isColliding)continue;
      else counter++;

      /* Define a box to be attached */
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = 0.05;
      primitive.dimensions[1] = 0.05;
      primitive.dimensions[2] = 0.05;

      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(pose);
      planning_scene.world.collision_objects.push_back(object);
  }
  planning_scene_diff_publisher->publish(planning_scene);
}

bool Scene::attachObject(){
  return true;
}
bool Scene::detachObject(){
  return true;
}
