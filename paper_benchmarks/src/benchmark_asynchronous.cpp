#include "paper_benchmarks/benchmark_asynchronous.hpp"
#include <thread>
#include <iostream>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

using namespace std::chrono_literals;

int number_of_test_cases = 5;

static struct runner{
  int counter = 0;
  std::mutex mtx;

  void increment(){
    std::lock_guard<std::mutex> lock(mtx);
    counter++;
  }

  int check(){
    std::lock_guard<std::mutex> lock(mtx);
    return counter;
  }
} runner2;

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

  std::thread t_1(main_thread_arm, pnp_1, "panda_1");
  std::thread t_2(main_thread_arm, pnp_2, "panda_2");

  rclcpp::spin(node);

  t_1.join();
  t_2.join();
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

    // if(update_scene_called_once){
      
    // }

    std::this_thread::sleep_for(1.0s);
    update_scene_called_once = true;
  }
}


void main_thread_arm(std::shared_ptr<primitive_pick_and_place> pnp, std::string robot_arm){

  pnp->home();
  pnp->open_gripper();

  moveit::planning_interface::MoveGroupInterface panda_arm(node, robot_arm);
  panda_arm.setMaxVelocityScalingFactor(0.50);
  panda_arm.setMaxAccelerationScalingFactor(0.50);
  panda_arm.setNumPlanningAttempts(5);
  panda_arm.setPlanningTime(1);

  moveit::core::RobotModelConstPtr kinematic_model = panda_arm.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = panda_arm.getCurrentState();

  arm_state arm_s(kinematic_model->getJointModelGroup(robot_arm));

  while (!update_scene_called_once)
  {
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER, "Size: %li", objs.size());

  int i = 0;

  RCLCPP_INFO(LOGGER, "[checkpoint] Starting execution");

  std::string curren_planning_robot;

  while(!objs.empty()){
    CollisionPlanningObject current_object;

    // change end effector position based on the robot available

    // we need to get position of the tool tip
    if (robot_arm.compare("panda_1") == 0)
    {
      geometry_msgs::msg::PoseStamped point_x = panda_arm.getCurrentPose("panda_1_leftfinger");
      // e.x = 0;
      // e.y = -0.5;
      // e.z = 1;
      e.x = point_x.pose.position.x;
      e.y = point_x.pose.position.y;
      e.z = point_x.pose.position.z;
      curren_planning_robot = "robot_1";
    }
    else //(!planned_for_panda_2)
    {
      geometry_msgs::msg::PoseStamped point_x = panda_arm.getCurrentPose("panda_2_leftfinger");
      // e.x = 0;
      // e.y = 0.5;
      // e.z = 1;
      e.x = point_x.pose.position.x;
      e.y = point_x.pose.position.y;
      e.z = point_x.pose.position.z;
      curren_planning_robot = "robot_2";
    }

    //objs.updatePoint(e);
    //current_object = objs.pop(curren_planning_robot, "random");

    if(objs.size() < 4)
    {
      auto message = std_msgs::msg::String();
      for(int i = 0; i < 8; i++){
        publisher_->publish(message);
      }
    }


    current_object = objs.pop(curren_planning_robot, "", e);

    auto object_id = current_object.collisionObject.id;
    RCLCPP_INFO(LOGGER, "Object: %s", object_id.c_str());

    // Check if the object is a box
    if (object_id.rfind("box", 0) != 0)
    {
      continue;
    }
 
    // plan for if the arm one is not busy
    tray_helper *active_tray;
    if (robot_arm.compare("panda_1") == 0)
    {
      if (colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)
        active_tray = &red_tray_1;
      else if (colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)
        active_tray = &blue_tray_1;
      else
        continue;
    }
    else //(!planned_for_panda_2)
    {
      if (colors[object_id].color.r == 1 && colors[object_id].color.g == 0 && colors[object_id].color.b == 0)
        active_tray = &red_tray_2;
      else if (colors[object_id].color.r == 0 && colors[object_id].color.g == 0 && colors[object_id].color.b == 1)
        active_tray = &blue_tray_2;
      else
        continue;
    }
    

    
    auto current_object_1 = std::move(current_object);
    
    //bool panda_1_success = executeTrajectory(pnp_1, current_object_1.collisionObject,active_tray);
    bool panda_1_success;

    if (robot_arm.compare("panda_1") == 0)
    {
      panda_1_success = advancedExecuteTrajectory(arm_s, panda_arm, kinematic_model, 
    kinematic_state, current_object_1.collisionObject,active_tray, 1);
    }
    else //(!planned_for_panda_2)
    {
      panda_1_success = advancedExecuteTrajectory(arm_s, panda_arm, kinematic_model, 
    kinematic_state, current_object_1.collisionObject,active_tray, 2);
    }
          
    if(!panda_1_success)
    {
      objs.push(current_object_1);
    }else{
      runner2.increment();
      RCLCPP_INFO(LOGGER, "[checkpoint] {%s} successful placing. Request to spawn a new cube ", curren_planning_robot.c_str());
            
      if(runner2.check() >= number_of_test_cases){
        RCLCPP_INFO(LOGGER, "[terminate]");
      }
                 
    }
  }

}


bool advancedExecuteTrajectory(arm_state &arm_1_state, moveit::planning_interface::MoveGroupInterface &panda_1_arm,  
  moveit::core::RobotModelConstPtr kinematic_model, moveit::core::RobotStatePtr kinematic_state, 
  moveit_msgs::msg::CollisionObject &object, tray_helper *tray, int s)
{

  RCLCPP_INFO(LOGGER, "Start execution of Object: %s", object.id.c_str());
  geometry_msgs::msg::Pose pose;

  // Pre Grasp
  arm_1_state.pose.position.x = object.pose.position.x;
  arm_1_state.pose.position.y = object.pose.position.y;
  arm_1_state.pose.position.z = object.pose.position.z + 0.25;

  arm_1_state.pose.orientation.x = object.pose.orientation.w;
  arm_1_state.pose.orientation.y = object.pose.orientation.z;
  arm_1_state.pose.orientation.z = 0;
  arm_1_state.pose.orientation.w = 0;

  bool executionSuccessful = false;

  RCLCPP_INFO(LOGGER, "Set from ik done ");
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting pregrasp execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "pregrasp planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        return false;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }


  // Pre Grasp
  arm_1_state.pose.position.z = object.pose.position.z + 0.1;

  executionSuccessful = false;
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting grasp execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "grasp planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        return false;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }


  // pre move
  if(s == 1){
    pnp_1->grasp_object(object);
  }else if(s == 2){
    pnp_2->grasp_object(object);
  }

  arm_1_state.pose.position.z = object.pose.position.z + 0.25;

  executionSuccessful = false;
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting premove execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "premove planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        continue;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }

  // Move
  arm_1_state.pose.position.x = tray->get_x();
  arm_1_state.pose.position.y = tray->get_y();
  arm_1_state.pose.position.z = 1.28 + tray->z * 0.05;

  arm_1_state.pose.orientation.x = 1;
  arm_1_state.pose.orientation.y = 0;

  executionSuccessful = false;
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting premove execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "premove planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        continue;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }
  // Put down
  arm_1_state.pose.position.z = 1.141 + tray->z * 0.05;

  executionSuccessful = false;
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting premove execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "premove planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        continue;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
    }
  }

  if(s == 1){
    pnp_1->release_object(object);
  }else if(s == 2){
    pnp_2->release_object(object);
  }
  // Post Move
  arm_1_state.pose.position.z = 1.28 + tray->z * 0.05;

  executionSuccessful = false;
  
  while (!executionSuccessful)
  {

    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, 0.1);

    RCLCPP_INFO(LOGGER, "Starting premove execution ");

    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_1_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm %d", s);
    }else{
      continue;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      bool success = (panda_1_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(success){
        RCLCPP_INFO(LOGGER, "premove planning successful");
      }
      if (!success || my_plan.trajectory_.joint_trajectory.points.size() == 0)
      {
        RCLCPP_INFO(LOGGER, "Plan did not succeed");
        continue;
      }
      executionSuccessful = panda_1_arm.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS;
      if(!executionSuccessful)
      {
        if(s == 1){
          pnp_1->release_object(object);
        }else if(s == 2){
          pnp_2->release_object(object);
        }
      }
    }
  }
  
  tray->next();

  return true;
}