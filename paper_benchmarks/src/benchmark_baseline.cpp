#include "paper_benchmarks/benchmark_baseline.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  pnp = std::make_shared<primitive_pick_and_place>(node,"panda_1");

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{ 
  rclcpp::Rate r(1);
  bool success = false;

  pnp->open_gripper();

  auto objs = pnp->getCollisionObjects();

  auto colors = pnp->getCollisionObjectColors();

  RCLCPP_INFO(LOGGER,"Size: %i",objs.size());

  geometry_msgs::msg::Pose pose;

  tray_helper blue_tray(4,4,0.11,-0.925);
  tray_helper red_tray(4,4,-0.425,-0.925);
  tray_helper* active_tray;

  for(auto obj : objs){
    auto active_object = obj.second;
    RCLCPP_INFO(LOGGER,"Object: %s",obj.first.c_str());

    //Check if the object is a box
    if(obj.first.rfind("box",0) != 0) continue;

    if(colors[obj.first].color.r == 1 && colors[obj.first].color.g == 0 && colors[obj.first].color.b == 0)active_tray = &red_tray;
    else if(colors[obj.first].color.r == 0 && colors[obj.first].color.g == 0 && colors[obj.first].color.b == 1)active_tray = &blue_tray;
    else continue;

    //Pre Grasp
    pose.position.x = active_object.pose.position.x;
    pose.position.y = active_object.pose.position.y;
    pose.position.z = active_object.pose.position.z + 0.25;

    pose.orientation.x = active_object.pose.orientation.w;
    pose.orientation.y = active_object.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    if(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute())
    {
      //Grasp
      pose.position.z = active_object.pose.position.z + 0.1;

      if(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute())
      {
        pnp->grasp_object(active_object);

        //Once grasped, no turning back! From now, ensure execution with while

        //Pre Move
        pose.position.z = active_object.pose.position.z + 0.25;

        while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
        {
          RCLCPP_INFO(LOGGER,"Try again");
        }

        //Move
        pose.position.x = active_tray->x_offset + active_tray->x * 0.06;
        pose.position.y = active_tray->y_offset + active_tray->y * 0.1;
        pose.position.z = 1.28 + active_tray->z * 0.05;

        pose.orientation.x = 1;
        pose.orientation.y = 0;

        while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
        {
          RCLCPP_INFO(LOGGER,"Try again 2");
        }
        // Put down
        pose.position.z = 1.13 + active_tray->z * 0.05;

        while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
        {
          RCLCPP_INFO(LOGGER,"Try again 3");
        }
        pnp->release_object(active_object);
        //Post Move
        pose.position.z = 1.28 + active_tray->z * 0.05; 

        while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
        {
          RCLCPP_INFO(LOGGER,"Try again 4");
        }
        active_tray->next();
        success = true;
      }
    }
    r.sleep();
  }
  

}