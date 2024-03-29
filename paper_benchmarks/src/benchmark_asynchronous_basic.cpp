#include "paper_benchmarks/benchmark_asynchronous.hpp"
#include <thread>
#include <iostream>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

using namespace std::chrono_literals;

int number_of_test_cases = 5;

static struct runner{
  int counter = 1;
  std::mutex mtx;

  void increment(){
    std::lock_guard<std::mutex> lock(mtx);
    counter++;
  }

  int check(){
    std::lock_guard<std::mutex> lock(mtx);
    return counter;
  }
} runner1;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("benchmark_asynchronous");

  node->declare_parameter("launchType", "randomDistance");
  node->declare_parameter("cubesToPick", 5);

  std::string distanceType = node->get_parameter("launchType").as_string();
  
  number_of_test_cases = node->get_parameter("cubesToPick").as_int();

  RCLCPP_INFO(LOGGER, "launch: %s", distanceType.c_str());

  pnp_1 = std::make_shared<primitive_pick_and_place>(node, "panda_1");
  pnp_2 = std::make_shared<primitive_pick_and_place>(node, "panda_2");

  publisher_ = node->create_publisher<std_msgs::msg::String>("spawnNewCube", 10);

  new std::thread(update_planning_scene);

  new std::thread(main_thread);

  rclcpp::spin(node);
  rclcpp::shutdown();
}

void update_planning_scene()
{

  while (true)
  {

    objMap = pnp_1->getCollisionObjects();
    colors = pnp_1->getCollisionObjectColors();

    for (auto &pair : objMap)
    {
      auto it = std::find(all_objects.begin(), all_objects.end(), pair.second.id);

      // not found in the object list
      if (it == all_objects.end())
      {
        all_objects.push_back(pair.second.id);

        CollisionPlanningObject new_object(pair.second, 0, 0);
        objs.push(new_object);

        RCLCPP_INFO(LOGGER, "New object detected. id: %s", pair.second.id.c_str());
      }
    }

    std::this_thread::sleep_for(1.0s);
    update_scene_called_once = true;
  }
}

void main_thread()
{
  rclcpp::Rate r(5);

  pnp_1->home();
  pnp_2->home();

  pnp_1->open_gripper();
  pnp_2->open_gripper();

  while (!update_scene_called_once)
  {
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER, "Size: %li", objs.size());

  int i = 0;

  RCLCPP_INFO(LOGGER, "[checkpoint] Starting execution");

  while (!objs.empty())
  {
    CollisionPlanningObject current_object;
    std::string curren_planning_robot = "robot_1";

    // start planning if atleast one of the arms are available
    if (!panda_1_busy || !panda_2_busy)
    {

      // change end effector position based on the robot available
      if (!panda_1_busy)
      {
        e.x = 0;
        e.y = -0.5;
        e.z = 1;
        curren_planning_robot = "robot_1";
      }
      else if (!panda_2_busy)
      {
        e.x = 0;
        e.y = 0.5;
        e.z = 1;
        curren_planning_robot = "robot_2";
      }

      objs.updatePoint(e);
      current_object = objs.pop(curren_planning_robot, "random");

      auto object_id = current_object.collisionObject.id;
      RCLCPP_INFO(LOGGER, "Object: %s", object_id.c_str());

      // Check if the object is a box
      if (object_id.rfind("box", 0) != 0)
      {
        continue;
      }

      bool panda_1_success = true;
      bool panda_2_success = true;

      // plan for if the arm one is not busy
      if (!panda_1_busy)
      {
        tray_helper *active_tray;
        if (colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)
          active_tray = &red_tray_1;
        else if (colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)
          active_tray = &blue_tray_1;
        else
          continue;

        new std::thread([&]()
                        {
          panda_1_busy = true;
          auto current_object_1 = std::move(current_object);
          bool panda_1_success = executeTrajectory(pnp_1, current_object_1.collisionObject,active_tray);
          
          if(!panda_1_success)
          {
            objs.push(current_object_1);
          }else{
            runner1.increment();
            RCLCPP_INFO(LOGGER, "[checkpoint] Robot 1 successful placing. Request to spawn a new cube ");
            
            if(runner1.check() >= number_of_test_cases)
              RCLCPP_INFO(LOGGER, "[terminate]");
          
            auto message = std_msgs::msg::String();
            publisher_->publish(message);
          }
          panda_1_busy = false; });
        
        std::this_thread::sleep_for(10.s);
      }

      //plan for if the arm one is not busy
      else if (!panda_2_busy)
      {
        tray_helper *active_tray;
        if (colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)
          active_tray = &red_tray_2;
        else if (colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)
          active_tray = &blue_tray_2;
        else
          continue;
        new std::thread([&]()
                        {
          panda_2_busy = true;
          auto current_object_2 = std::move(current_object);
          panda_2_success = executeTrajectory(pnp_2, current_object_2.collisionObject,active_tray);
          
          if(!panda_2_success)
          {
            objs.push(current_object_2);
          }else{
            runner1.increment();
            RCLCPP_INFO(LOGGER, "[checkpoint] Robot 2 successful placing. Request to spawn a new cube ");
            
            if(runner1.check() >= number_of_test_cases)
              RCLCPP_INFO(LOGGER, "[terminate]");

            auto message = std_msgs::msg::String();
            publisher_->publish(message);
          }
          panda_2_busy = false; });
        std::this_thread::sleep_for(0.1s);
      }
    }
    r.sleep();
  }

  RCLCPP_INFO(LOGGER, "Execution completed");
}

bool executeTrajectory(std::shared_ptr<primitive_pick_and_place> pnp, moveit_msgs::msg::CollisionObject &object, tray_helper *tray)
{
  static int thread_local pregrasp_planning_retries = 0;
  static int thread_local pregrasp_executing_retries = 0;
  static int thread_local grasp_planning_retries = 0;
  static int thread_local grasp_executing_retries = 0;

  RCLCPP_INFO(LOGGER, "Start execution of Object: %s", object.id.c_str());
  geometry_msgs::msg::Pose pose;
  pnp->open_gripper();

  std::this_thread::sleep_for(0.2s);

  // Pre Grasp
  pose.position.x = object.pose.position.x;
  pose.position.y = object.pose.position.y;
  pose.position.z = object.pose.position.z + 0.25;

  pose.orientation.x = object.pose.orientation.w;
  pose.orientation.y = object.pose.orientation.z;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again pre grasp failed");
    if (!pnp->is_plan_successful())
    {
      RCLCPP_ERROR(LOGGER, "Pre grasp planner failed");
      if(pregrasp_planning_retries >= 2)
      {
        pregrasp_planning_retries = 0;
        return false;
      }
      else
      {
        pregrasp_planning_retries++;
        RCLCPP_ERROR(LOGGER, "Retrying pregrasp planning");
      }
    }
    if(!pnp->is_execution_successful())
    {
      RCLCPP_ERROR(LOGGER, "Pre grasp execution failed");
      if(pregrasp_executing_retries >= 2)
      {
        pregrasp_executing_retries = 0;
        return false;
      }
      else
      {
        pregrasp_executing_retries++;
        RCLCPP_ERROR(LOGGER, "Retrying pregrasp execution");
      }
    }
  }

  pnp->set_default();
  
  // Grasp
  pose.position.z = object.pose.position.z + 0.1;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again grasp failed");
    if (!pnp->is_plan_successful())
    {
      RCLCPP_ERROR(LOGGER, "Grasp planner failed");
      if(grasp_planning_retries >= 2)
      {
        grasp_planning_retries = 0;
        return false;
      }
      else
      {
        grasp_planning_retries++;
        RCLCPP_ERROR(LOGGER, "Retrying grasp planning");
      }
    }
    if(!pnp->is_execution_successful())
    {
      RCLCPP_ERROR(LOGGER, "Grasp execution failed");
      if(grasp_executing_retries >= 2)
      {
        grasp_executing_retries = 0;
        return false;
      }
      else
      {
        grasp_executing_retries++;
        RCLCPP_ERROR(LOGGER, "Retrying grasp executing");
      }
    }
  }

  pnp->set_default();
  
  pnp->grasp_object(object);
  RCLCPP_INFO(LOGGER, "Grasping object with ID %s", object.id.c_str());

  // Once grasped, no turning back! From now, ensure execution with while

  // Pre Move
  pose.position.z = object.pose.position.z + 0.25;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again pre move failed");
    if(!pnp->is_plan_successful())
    {
      RCLCPP_INFO(LOGGER, "Pre move planning failed +++++");
    }
    if(!pnp->is_execution_successful())
    {
      RCLCPP_INFO(LOGGER, "Pre move execution failed ------"); 
    }
  }

  // Move
  pose.position.x = tray->get_x();
  pose.position.y = tray->get_y();
  pose.position.z = 1.28 + tray->z * 0.05;

  pose.orientation.x = 1;
  pose.orientation.y = 0;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again move failed");
  }
  // Put down
  pose.position.z = 1.141 + tray->z * 0.05;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again put down failed");
  }
  pnp->release_object(object);
  // Post Move
  pose.position.z = 1.28 + tray->z * 0.05;

  while (!(pnp->set_joint_values_from_pose(pose) && pnp->generate_plan() && pnp->execute()))
  {
    RCLCPP_INFO(LOGGER, "Try again post move failed");
  }
  tray->next();

  return true;
}
