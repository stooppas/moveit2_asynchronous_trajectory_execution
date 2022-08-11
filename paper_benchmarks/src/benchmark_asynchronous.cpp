#include "paper_benchmarks/benchmark_asynchronous.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_asynchronous");

  pnp_1 = std::make_shared<primitive_pick_and_place>(node,"panda_1");
  pnp_2 = std::make_shared<primitive_pick_and_place>(node,"panda_2");

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{ 
  rclcpp::Rate r(5);

  auto objs = pnp_1->getCollisionObjects();

  auto colors = pnp_1->getCollisionObjectColors();

  RCLCPP_INFO(LOGGER,"Size: %i",objs.size());

  int i = 0;

  auto it = objs.begin();

  while(it != objs.end()){
    if(!panda_1_busy || !panda_2_busy){
      auto active_object = it->second;
      auto object_id = it->first;
      RCLCPP_INFO(LOGGER,"Object: %s",object_id.c_str());

      //Check if the object is a box
      if(object_id.rfind("box",0) != 0){
        it++;
        continue;
      }

      if(!panda_1_busy)
      {
        tray_helper* active_tray;
        if(colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)active_tray = &red_tray_1;
        else if(colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)active_tray = &blue_tray_1;
        else continue;
        new std::thread([&](){
          panda_1_busy = true;
          executeTrajectory(pnp_1,active_object,active_tray);
          panda_1_busy = false;
        });
        it++;
      }
      else if (!panda_2_busy)
      {
        tray_helper* active_tray;
        if(colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)active_tray = &red_tray_2;
        else if(colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)active_tray = &blue_tray_2;
        else continue;
        new std::thread([&](){
          panda_2_busy = true;
          executeTrajectory(pnp_2,active_object,active_tray);
          panda_2_busy = false;
        });
        it++;
      }
    }
    r.sleep();
  }
}
bool executeTrajectory(std::shared_ptr<primitive_pick_and_place> pnp,moveit_msgs::msg::CollisionObject& object,tray_helper* tray)
{
  geometry_msgs::msg::Pose pose;
  pnp->open_gripper();

  //Pre Grasp
  pose.position.x = object.pose.position.x;
  pose.position.y = object.pose.position.y;
  pose.position.z = object.pose.position.z + 0.25;

  pose.orientation.x = object.pose.orientation.w;
  pose.orientation.y = object.pose.orientation.z;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again");
  }
  //Grasp
  pose.position.z = object.pose.position.z + 0.1;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again");
  }
  pnp->grasp_object(object);
  RCLCPP_INFO(LOGGER,"Grasping object with ID %s",object.id.c_str());

  //Once grasped, no turning back! From now, ensure execution with while

  //Pre Move
  pose.position.z = object.pose.position.z + 0.25;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again");
  }

  //Move
  pose.position.x = tray->x_offset + tray->x * 0.06;
  pose.position.y = tray->y_offset + tray->y * 0.1;
  pose.position.z = 1.28 + tray->z * 0.05;

  pose.orientation.x = 1;
  pose.orientation.y = 0;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again 2");
  }
  // Put down
  pose.position.z = 1.13 + tray->z * 0.05;

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again 3");
  }
  pnp->release_object(object);
  //Post Move
  pose.position.z = 1.28 + tray->z * 0.05; 

  while(!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER,"Try again 4");
  }
  tray->next();

  return true;
}