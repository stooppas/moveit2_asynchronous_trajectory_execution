#include "paper_benchmarks/benchmark_asynchronous.hpp"
#include <thread>
#include <iostream>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

using namespace std::chrono_literals;

void add_objects_to_planning_scene();

enum ROBOT_STATE{
  PREGRASP,
  GRASP,
  PREMOVE,
  MOVE,
  PUTDOWN,
  POSTMOVE
};

ROBOT_STATE robot_1_state;
ROBOT_STATE robot_2_state;

volatile int number_of_test_cases = 5;

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

  new std::thread(add_objects_to_planning_scene);

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

    std::this_thread::sleep_for(1.0s);
    update_scene_called_once = true;
  }
}


void add_objects_to_planning_scene(){

  while(true){

    while (!update_scene_called_once)
    {
      std::this_thread::sleep_for(1.0s);
    }

    if(robot_1_state == ROBOT_STATE::PREGRASP || robot_1_state == ROBOT_STATE::GRASP  || robot_1_state == ROBOT_STATE::PREMOVE|| 
       robot_2_state == ROBOT_STATE::PREGRASP || robot_2_state == ROBOT_STATE::GRASP || robot_2_state == ROBOT_STATE::PREMOVE )
    {
      std::this_thread::sleep_for(0.5s);
      continue;
    } 

    int amount_to_appear = std::min( number_of_test_cases - int(objs.size()) - runner2.check(), 8);

    if(objs.size() < 6)
    {
      auto message = std_msgs::msg::String();
      for(int i = 0; i < amount_to_appear; i++){
        publisher_->publish(message);
      }
      std::this_thread::sleep_for(10.0s);

    }else{
      std::this_thread::sleep_for(3.0s);
    }

    if (amount_to_appear == 0)
      break;

  }
}


void main_thread_arm(std::shared_ptr<primitive_pick_and_place> pnp, std::string robot_arm){

  pnp->home();
  pnp->open_gripper();

  moveit::planning_interface::MoveGroupInterface panda_arm(node, robot_arm);
  panda_arm.setMaxVelocityScalingFactor(0.50);
  panda_arm.setMaxAccelerationScalingFactor(0.50);
  panda_arm.setNumPlanningAttempts(20);
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

    arm_s.object = current_object_1;
    
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

bool plan_and_move(arm_state &arm_1_state, ROBOT_STATE movement, moveit::core::RobotStatePtr kinematic_state,
                   double timeout, moveit::planning_interface::MoveGroupInterface &panda_arm, tray_helper *active_tray);

bool advancedExecuteTrajectory(arm_state &arm_1_state, moveit::planning_interface::MoveGroupInterface &panda_1_arm,  
  moveit::core::RobotModelConstPtr kinematic_model, moveit::core::RobotStatePtr kinematic_state, 
  moveit_msgs::msg::CollisionObject &object, tray_helper *tray, int s)
{

  RCLCPP_INFO(LOGGER, "Start execution of Object: %s", object.id.c_str());
  geometry_msgs::msg::Pose pose;

  std::string robot = "robot_1";

  bool success = plan_and_move(arm_1_state, ROBOT_STATE::PREGRASP, kinematic_state, 1, panda_1_arm, tray);
  
  if (!success)
  {
    return false;
  }

  success = plan_and_move(arm_1_state, ROBOT_STATE::GRASP, kinematic_state, 1, panda_1_arm, tray);
  
  if (!success)
  {
    return false;
  }

  if(s == 1){
    pnp_1->grasp_object(arm_1_state.object.collisionObject);
  }else if(s == 2){
    pnp_2->grasp_object(arm_1_state.object.collisionObject);
  }

  plan_and_move(arm_1_state, ROBOT_STATE::PREMOVE, kinematic_state, 1, panda_1_arm, tray);

  plan_and_move(arm_1_state, ROBOT_STATE::MOVE, kinematic_state, 1, panda_1_arm, tray);

  plan_and_move(arm_1_state, ROBOT_STATE::PUTDOWN, kinematic_state, 1, panda_1_arm, tray);

  if(s == 1){
    pnp_1->release_object(arm_1_state.object.collisionObject);
  }else if(s == 2){
    pnp_2->release_object(arm_1_state.object.collisionObject);
  } 
  
  plan_and_move(arm_1_state, ROBOT_STATE::POSTMOVE, kinematic_state, 1, panda_1_arm, tray);

  return true;
}



bool plan_and_move(arm_state &arm_1_state, ROBOT_STATE movement, moveit::core::RobotStatePtr kinematic_state,
                   double timeout, moveit::planning_interface::MoveGroupInterface &panda_arm, tray_helper *active_tray)
{
  thread_local int cache_1;

  if (movement == ROBOT_STATE::PREGRASP)
  {
    RCLCPP_INFO(LOGGER, "[Movement type pregrasp]");
    arm_1_state.pose.position.x = arm_1_state.object.collisionObject.pose.position.x;
    arm_1_state.pose.position.y = arm_1_state.object.collisionObject.pose.position.y;
    arm_1_state.pose.position.z = arm_1_state.object.collisionObject.pose.position.z + 0.25;

    arm_1_state.pose.orientation.x = arm_1_state.object.collisionObject.pose.orientation.w;
    arm_1_state.pose.orientation.y = arm_1_state.object.collisionObject.pose.orientation.z;
    arm_1_state.pose.orientation.z = 0;
    arm_1_state.pose.orientation.w = 0;
  }
  else if (movement == ROBOT_STATE::GRASP)
  {
    RCLCPP_INFO(LOGGER, "[Movement type grasp]");
    arm_1_state.pose.position.z = arm_1_state.object.collisionObject.pose.position.z + 0.1;
  }
  else if (movement == ROBOT_STATE::PREMOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type pre move]");
    arm_1_state.pose.position.z = arm_1_state.object.collisionObject.pose.position.z + 0.25;
  }
  else if (movement == ROBOT_STATE::MOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type move]");
    arm_1_state.pose.position.x = active_tray->get_x();
    arm_1_state.pose.position.y = active_tray->get_y();
    arm_1_state.pose.position.z = 1.28 + active_tray->z * 0.05;

    arm_1_state.pose.orientation.x = 1;
    arm_1_state.pose.orientation.y = 0;

    cache_1 = active_tray->z * 0.05;

    active_tray->next();
  }
  else if (movement == ROBOT_STATE::PUTDOWN)
  {
    RCLCPP_INFO(LOGGER, "[Movement type putdown move]");
    arm_1_state.pose.position.z = 1.141 + cache_1;
  }
  else if (movement == ROBOT_STATE::POSTMOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type post move]");
    arm_1_state.pose.position.z = 1.28 + cache_1;
  }

  bool executionSuccessful = false;
  int counter = 0;

  while (!executionSuccessful)
  {
    bool a_bot_found_ik = kinematic_state->setFromIK(arm_1_state.arm_joint_model_group, arm_1_state.pose, timeout);
      
    if (a_bot_found_ik)
    {
      kinematic_state->copyJointGroupPositions(arm_1_state.arm_joint_model_group, arm_1_state.arm_joint_values);
      panda_arm.setJointValueTarget(arm_1_state.arm_joint_names, arm_1_state.arm_joint_values);
      RCLCPP_INFO(LOGGER, "IK found for arm 1");
    }

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (a_bot_found_ik)
    {
      executionSuccessful = panda_arm.move() == moveit::core::MoveItErrorCode::SUCCESS;
      if(!executionSuccessful){
        objs.push(arm_1_state.object);
        //pnp_1->open_gripper();
        return false;
      }
    }
    else
    {
      if (movement == ROBOT_STATE::PREGRASP || movement == ROBOT_STATE::GRASP)
      {
        objs.push(arm_1_state.object);
        return false;
      }
    }
  }
  return true;
}