#include "paper_benchmarks/benchmark_synchronous.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  pnp_1 = std::make_shared<primitive_pick_and_place>(node,"panda_1");
  pnp_2 = std::make_shared<primitive_pick_and_place>(node,"panda_2");
  pnp_dual = std::make_shared<primitive_pick_and_place>(node,"dual_arm");

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void main_thread()
{ 
  srand(time(0));
  rclcpp::Rate r(1);
  bool success = false;

  pnp_1->open_gripper();
  pnp_2->open_gripper();

  auto objMap = pnp_dual->getCollisionObjects();

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  for(const auto &obj : objMap)objs.push_back(obj.second);
  std::random_shuffle(objs.begin(),objs.end());

  auto colors = pnp_dual->getCollisionObjectColors();

  RCLCPP_INFO(LOGGER,"Size: %i",objs.size());

  geometry_msgs::msg::Pose pose;

  tray_helper blue_tray(6,3,0.11,-0.925,0.06,0.1,true);
  tray_helper red_tray(6,3,-0.425,-0.925,0.06,0.1,true);
  tray_helper* active_tray;

  auto it = objs.begin();
  /*

  while(it != objs.end()){
    auto active_object = *it;
    auto object_id = (*it).id;
    RCLCPP_INFO(LOGGER,"Object: %s",object_id.c_str());

    //Check if the object is a box
    if(active_object.id.rfind("box",0) != 0)
    {
      RCLCPP_INFO(LOGGER,"Skipping");
      continue;
    }

    if(colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)active_tray = &red_tray;
    else if(colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)active_tray = &blue_tray;
    else continue;

    //Pre Grasp
    pose.position.x = active_object.pose.position.x;
    pose.position.y = active_object.pose.position.y;
    pose.position.z = active_object.pose.position.z + 0.25;

    pose.orientation.x = active_object.pose.orientation.w;
    pose.orientation.y = active_object.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0; 

    double timeout = 0.1;
    bool panda_1_found_ik = pnp_1->set_joint_values_from_pose(pose);

    auto newIt = it;
    bool found;
    while(newIt != objs.end() && !found){
      geometry_msgs::msg::Pose pose_2;

      pose_2.position.x = (*newIt).pose.position.x;
      pose_2.position.y = (*newIt).pose.position.y;
      pose_2.position.z = (*newIt).pose.position.z + 0.25;

      pose_2.orientation.x = (*newIt).pose.orientation.w;
      pose_2.orientation.y = (*newIt).pose.orientation.z;
      pose_2.orientation.z = 0;
      pose_2.orientation.w = 0; 

      if(pnp_2->set_joint_values_from_pose(pose)){
        geometry_msgs::msg::Pose pose_combined;
        found = pnp_dual->set_joint_values_from_pose(pose_combined);
      }
    }

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }

    //Grasp
    pose.position.z = active_object.pose.position.z + 0.1;

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    pnp->grasp_object(active_object);

    //Pre Move
    pose.position.z = active_object.pose.position.z + 0.25;

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }

    //Move
    pose.position.x = active_tray->get_x();
    pose.position.y = active_tray->get_y();
    pose.position.z = 1.28 + active_tray->z * 0.05;

    pose.orientation.x = 1;
    pose.orientation.y = 0;

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    // Put down
    pose.position.z = 1.141 + active_tray->z * 0.05;

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    pnp->release_object(active_object);
    //Post Move
    pose.position.z = 1.28 + active_tray->z * 0.05; 

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    active_tray->next();
    success = true;
    r.sleep();
  }*/
  
  RCLCPP_INFO(LOGGER,"Finished");
}