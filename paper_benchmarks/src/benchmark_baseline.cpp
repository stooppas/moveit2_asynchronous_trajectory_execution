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
  srand(time(0));
  rclcpp::Rate r(1);
  bool success = false;

  pnp->home();

  pnp->open_gripper();

  auto objMap = pnp->getCollisionObjects();

  std::vector<moveit_msgs::msg::CollisionObject> objs;
  for(const auto &obj : objMap)objs.push_back(obj.second);
  std::random_shuffle(objs.begin(),objs.end());

  auto colors = pnp->getCollisionObjectColors();

  RCLCPP_INFO(LOGGER,"Size: %i",objs.size());

  geometry_msgs::msg::Pose pose;

  tray_helper blue_tray(6,3,0.11,-0.925,0.06,0.1,true);
  tray_helper red_tray(6,3,-0.425,-0.925,0.06,0.1,true);
  tray_helper* active_tray;

  for(auto obj : objs){
    RCLCPP_INFO(LOGGER,"Object: %s",obj.id.c_str());

    //Check if the object is a box
    if(obj.id.rfind("box",0) != 0)
    {
      RCLCPP_INFO(LOGGER,"Skipping");
      continue;
    }

    if(colors[obj.id].color.r == 1 && colors[obj.id].color.g == 0 && colors[obj.id].color.b == 0)active_tray = &red_tray;
    else if(colors[obj.id].color.r == 0 && colors[obj.id].color.g == 0 && colors[obj.id].color.b == 1)active_tray = &blue_tray;
    else continue;

    //Pre Grasp
    pose.position.x = obj.pose.position.x;
    pose.position.y = obj.pose.position.y;
    pose.position.z = obj.pose.position.z + 0.25;

    pose.orientation.x = obj.pose.orientation.w;
    pose.orientation.y = obj.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    tf2::Quaternion q_orig, q_rot, q_new;

    tf2::convert(pose.orientation,q_orig);

    tf2::Matrix3x3 m(q_orig);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);


    int attempts = 1;
    float offset = 0;


    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
      /*
      offset = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/0.05)) - 0.025;
      pose.position.x = obj.pose.position.x + offset*sin(yaw);
      pose.position.y = obj.pose.position.y + offset*cos(yaw);
      //Rotate Gripper
      /*
      tf2::Quaternion q_orig, q_rot, q_new;

      tf2::convert(pose.orientation,q_orig);

      q_rot.setRPY(0,0,(attempts++%4) *1.57079633);

      q_new = q_rot*q_orig;
      q_new.normalize();
      tf2::convert(q_new,pose.orientation);
      */
    }

    //Grasp
    pose.position.z = obj.pose.position.z + 0.1;

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    pnp->grasp_object(obj);

    //Pre Move
    pose.position.z = obj.pose.position.z + 0.25;

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
    pnp->release_object(obj);
    //Post Move
    pose.position.z = 1.28 + active_tray->z * 0.05; 

    while(!(pnp->set_joint_values_from_pose(pose) && pnp->plan_and_execute()))
    {
      RCLCPP_INFO(LOGGER,"Retrying");
    }
    active_tray->next();
    success = true;
    r.sleep();
  }
  
  RCLCPP_INFO(LOGGER,"Finished");
}