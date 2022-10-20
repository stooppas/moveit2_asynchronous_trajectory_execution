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
  auto planning_interface = new moveit::planning_interface::PlanningSceneInterface();

  srand(time(0));
  
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<moveit_msgs::msg::ObjectColor> object_colors;

  int counter = 1;
  float min_x = -0.35;
  float max_x = 0.35;
  float min_y = 0.25;
  float max_y = -0.25;
  while(counter < 20){
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "base";
      object.id = "box_" + std::to_string(counter);

      /* A default pose */
      geometry_msgs::msg::Pose pose;
      pose.position.x = min_x + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_x-min_x)));
      pose.position.y = min_y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max_y-min_y)));
      pose.position.z = 1.026;
      pose.orientation.z = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));;
      
      bool isColliding = false;

      for(auto object : collision_objects){
        double len = sqrt(pow(pose.position.x - object.primitive_poses[0].position.x,2) + pow(pose.position.y - object.primitive_poses[0].position.y,2));
        if(len < 0.1)
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

      moveit_msgs::msg::ObjectColor color;
      color.id = object.id;

      if(counter%2){
        color.color.r = 1;
        color.color.g = 0;
        color.color.b = 0;
        color.color.a = 1;
      }else{
        color.color.r = 0;
        color.color.g = 0;
        color.color.b = 1;
        color.color.a = 1;
      }

      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(pose);
      collision_objects.push_back(object);
      object_colors.push_back(color);
  }
  planning_interface->addCollisionObjects(collision_objects,object_colors);
}

bool Scene::attachObject(){
  return true;
}
bool Scene::detachObject(){
  return true;
}
