#include "paper_benchmarks/benchmark_synchronous.hpp"
#include "paper_benchmarks/cube_selector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
} runner1;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("benchmark_synchronous");

  node->declare_parameter("cubesToPick", 5);

  number_of_test_cases = node->get_parameter("cubesToPick").as_int();

  pnp_1 = std::make_shared<primitive_pick_and_place>(node, "panda_1");
  pnp_2 = std::make_shared<primitive_pick_and_place>(node, "panda_2");
  pnp_dual = std::make_shared<primitive_pick_and_place>(node, "dual_arm");

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

    objMap = pnp_dual->getCollisionObjects();
    colors = pnp_dual->getCollisionObjectColors();

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
  RCLCPP_INFO(LOGGER, "[Starting]");

  pnp_1->open_gripper();
  pnp_2->open_gripper();

  moveit::planning_interface::MoveGroupInterface panda_1_arm(node, "panda_1");
  moveit::planning_interface::MoveGroupInterface panda_2_arm(node, "panda_2");
  moveit::planning_interface::MoveGroupInterface dual_arm(node, "dual_arm");
  dual_arm.setMaxVelocityScalingFactor(0.50);
  dual_arm.setMaxAccelerationScalingFactor(0.50);
  dual_arm.setNumPlanningAttempts(5);
  dual_arm.setPlanningTime(1);

  RCLCPP_INFO(LOGGER, "[Initialized]");

  moveit::core::RobotModelConstPtr kinematic_model = panda_1_arm.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = dual_arm.getCurrentState();

  dual_arm_state arm_system(
      arm_state(kinematic_model->getJointModelGroup("panda_1")),
      arm_state(kinematic_model->getJointModelGroup("panda_2")));

  RCLCPP_INFO(LOGGER, "[Go to go]");

  srand(time(0));
  rclcpp::Rate r(1);
  bool success = false;

  while (!update_scene_called_once)
  {
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER, "Size: %li", objs.size());

  RCLCPP_INFO(LOGGER, "[checkpoint]");

  tray_helper blue_tray(6, 3, 0.11, -0.925, 0.06, 0.1, true);
  tray_helper red_tray(6, 3, -0.425, -0.925, 0.06, 0.1, true);
  tray_helper *active_tray_arm_1;
  tray_helper *active_tray_arm_2;

  RCLCPP_INFO(LOGGER, "Finished");

  int counter = 0;

  while (!objs.empty())
  {
    counter++;
    RCLCPP_INFO(LOGGER, "[starting pick and place]");

    //arm_system.arm_1.object = objs.pop("", "random");
    //arm_system.arm_2.object = objs.pop("", "random");
    
    int amount_to_appear = std::min( number_of_test_cases - int(objs.size()) - runner1.check(), 8);

    if(objs.size() < 6)
    {
      auto message = std_msgs::msg::String();
      for(int i = 0; i < amount_to_appear; i++){
        publisher_->publish(message);
      }
    }

    geometry_msgs::msg::PoseStamped point_x = dual_arm.getCurrentPose("panda_1_leftfinger");
    e.x = point_x.pose.position.x;
    e.y = point_x.pose.position.y;
    e.z = point_x.pose.position.z;
    
    bool execute_one =  objs.get_execute_one(); 
    
    arm_system.arm_1.object = objs.pop("robot_1", "", e);

    execute_one = execute_one | objs.empty(); // if last item was the one picked


    RCLCPP_INFO(LOGGER, "[object id %s ]", arm_system.arm_1.object.collisionObject.id.c_str());
    RCLCPP_INFO(LOGGER, "Next tray selection");

    auto object_1_id = arm_system.arm_1.object.collisionObject.id;

    RCLCPP_INFO(LOGGER, "Size colors %li", colors.size());

    std::string robot_1_color;
    std::string robot_2_color;

    if (colors[object_1_id].color.r == 1 && colors[object_1_id].color.g == 0 && colors[object_1_id].color.b == 0){
      active_tray_arm_1 = &red_tray_1;
      robot_1_color = "RED";
    }
    else if (colors[object_1_id].color.r == 0 && colors[object_1_id].color.g == 0 && colors[object_1_id].color.b == 1){
      active_tray_arm_1 = &blue_tray_1;
      robot_1_color = "BLUE";
    }
    else{
      continue;
    }

    if(!execute_one){
      point_x = dual_arm.getCurrentPose("panda_2_leftfinger");

      e.x = point_x.pose.position.x;
      e.y = point_x.pose.position.y;
      e.z = point_x.pose.position.z;

      arm_system.arm_2.object = objs.pop("robot_2", "", e);

      RCLCPP_INFO(LOGGER, "[object id %s ]", arm_system.arm_2.object.collisionObject.id.c_str());

      auto object_2_id = arm_system.arm_2.object.collisionObject.id;

      if (colors[object_2_id].color.r == 1 && colors[object_2_id].color.g == 0 && colors[object_2_id].color.b == 0){
        active_tray_arm_2 = &red_tray_2;
        robot_2_color = "RED";
      }
      else if (colors[object_2_id].color.r == 0 && colors[object_2_id].color.g == 0 && colors[object_2_id].color.b == 1){
        active_tray_arm_2 = &blue_tray_2;
        robot_2_color = "BLUE";
      }
      else
        continue;
    }
    
    bool success = plan_and_move(arm_system, Movement::PREGRASP, kinematic_state, 1, dual_arm,
                                 active_tray_arm_1, active_tray_arm_2, execute_one, false, false);
    if (!success)
    {
      continue;
    }

    // pnp_1->open_gripper();
    // pnp_2->open_gripper();

    success = plan_and_move(arm_system, Movement::GRASP, kinematic_state, 1, dual_arm,
                            active_tray_arm_1, active_tray_arm_2, execute_one, false, false);
    if (!success)
    {
      continue;
    }

    // from here onwards we cannot fail since the object is attached
    pnp_1->grasp_object(arm_system.arm_1.object.collisionObject);
    
    if(!execute_one){
      pnp_2->grasp_object(arm_system.arm_2.object.collisionObject);
    }

    plan_and_move(arm_system, Movement::PREMOVE, kinematic_state, 1, dual_arm,
                  active_tray_arm_1, active_tray_arm_2, execute_one, false, false);

    plan_and_move(arm_system, Movement::MOVE, kinematic_state, 1, dual_arm,
                  active_tray_arm_1, active_tray_arm_2, execute_one, false, false);

    // Task differentiation based on the color
    plan_and_move(arm_system, Movement::RANDOM_1, kinematic_state, 1, dual_arm,
                  active_tray_arm_1, active_tray_arm_2, execute_one, robot_1_color.compare("BLUE") == 0, robot_2_color.compare("BLUE") == 0);

    //plan_and_move(arm_system, Movement::RANDOM_1, kinematic_state, 1, dual_arm,
    //              active_tray_arm_1, active_tray_arm_2, execute_one, robot_1_color.compare("BLUE") == 0, robot_2_color.compare("BLUE") == 0);
 

    plan_and_move(arm_system, Movement::PUTDOWN, kinematic_state, 1, dual_arm,
                  active_tray_arm_1, active_tray_arm_2, execute_one, false, false);

    pnp_1->release_object(arm_system.arm_1.object.collisionObject);

    if(!execute_one){
      pnp_2->release_object(arm_system.arm_2.object.collisionObject);
    }

    plan_and_move(arm_system, Movement::POSTMOVE, kinematic_state, 1, dual_arm,
                  active_tray_arm_1, active_tray_arm_2, execute_one, false, false);

   
    runner1.increment();
    if(!execute_one){
      runner1.increment();
    }
    RCLCPP_INFO(LOGGER, "[checkpoint] Robot 1 successful placing. Request to spawn a new cube ");
    RCLCPP_INFO(LOGGER, "[checkpoint] Robot 2 successful placing. Request to spawn a new cube ");

    if(runner1.check() >= number_of_test_cases){
      RCLCPP_INFO(LOGGER, "[terminate]");
    }
    
  }
}

bool plan_and_move(dual_arm_state &arm_system, Movement movement, moveit::core::RobotStatePtr kinematic_state,
                   double timeout, moveit::planning_interface::MoveGroupInterface &dual_arm, tray_helper *active_tray_arm_1,
                   tray_helper *active_tray_arm_2, bool executeOneOnly, bool left, bool right)
{
  static int cache_1;
  static int cache_2;

  if (movement == Movement::PREGRASP)
  {
    RCLCPP_INFO(LOGGER, "[Movement type pregrasp]");
    arm_system.arm_1.pose.position.x = arm_system.arm_1.object.collisionObject.pose.position.x;
    arm_system.arm_1.pose.position.y = arm_system.arm_1.object.collisionObject.pose.position.y;
    arm_system.arm_1.pose.position.z = arm_system.arm_1.object.collisionObject.pose.position.z + 0.25;

    arm_system.arm_1.pose.orientation.x = arm_system.arm_1.object.collisionObject.pose.orientation.w;
    arm_system.arm_1.pose.orientation.y = arm_system.arm_1.object.collisionObject.pose.orientation.z;
    arm_system.arm_1.pose.orientation.z = 0;
    arm_system.arm_1.pose.orientation.w = 0;

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.x = arm_system.arm_2.object.collisionObject.pose.position.x;
      arm_system.arm_2.pose.position.y = arm_system.arm_2.object.collisionObject.pose.position.y;
      arm_system.arm_2.pose.position.z = arm_system.arm_2.object.collisionObject.pose.position.z + 0.25;

      arm_system.arm_2.pose.orientation.x = arm_system.arm_2.object.collisionObject.pose.orientation.w;
      arm_system.arm_2.pose.orientation.y = arm_system.arm_2.object.collisionObject.pose.orientation.z;
      arm_system.arm_2.pose.orientation.z = 0;
      arm_system.arm_2.pose.orientation.w = 0;
    }
  }
  else if (movement == Movement::GRASP)
  {
    RCLCPP_INFO(LOGGER, "[Movement type grasp]");
    arm_system.arm_1.pose.position.z = arm_system.arm_1.object.collisionObject.pose.position.z + 0.1;

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.z = arm_system.arm_2.object.collisionObject.pose.position.z + 0.1;
    }
  }
  else if (movement == Movement::PREMOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type pre move]");
    arm_system.arm_1.pose.position.z = arm_system.arm_1.object.collisionObject.pose.position.z + 0.25;

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.z = arm_system.arm_2.object.collisionObject.pose.position.z + 0.25;
    }
  }
  else if (movement == Movement::MOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type move]");
    arm_system.arm_1.pose.position.x = active_tray_arm_1->get_x();
    arm_system.arm_1.pose.position.y = active_tray_arm_1->get_y();
    arm_system.arm_1.pose.position.z = 1.28 + active_tray_arm_1->z * 0.05;

    arm_system.arm_1.pose.orientation.x = 1;
    arm_system.arm_1.pose.orientation.y = 0;

    cache_1 = active_tray_arm_1->z * 0.05;

    active_tray_arm_1->next();

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.x = active_tray_arm_2->get_x();
      arm_system.arm_2.pose.position.y = active_tray_arm_2->get_y();
      arm_system.arm_2.pose.position.z = 1.28 + active_tray_arm_2->z * 0.05;

      arm_system.arm_2.pose.orientation.x = 1;
      arm_system.arm_2.pose.orientation.y = 0;

      cache_2 = active_tray_arm_2->z * 0.05;

      active_tray_arm_2->next();
    }
  }
  else if (movement == Movement::PUTDOWN)
  {
    RCLCPP_INFO(LOGGER, "[Movement type putdown move]");

    arm_system.arm_1.pose.position.z = 1.141 + cache_1;

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.z = 1.141 + cache_2;
    }
  }
  else if (movement == Movement::POSTMOVE)
  {
    RCLCPP_INFO(LOGGER, "[Movement type post move]");

    arm_system.arm_1.pose.position.z = 1.28 + cache_1;

    if(!executeOneOnly){
      arm_system.arm_2.pose.position.z = 1.28 + cache_2;
    }
  }
  else if (movement == Movement::RANDOM_1)
  {
    if(left){
      RCLCPP_INFO(LOGGER, "[Movement type random 1]");
      arm_system.arm_1.pose.position.z = 1.48 + cache_2;
    }

    if(right){
      arm_system.arm_2.pose.position.z = 1.48 + cache_2;
    }

    if(left || right){
      std::this_thread::sleep_for(5.0s);
    }
  }

  else if (movement == Movement::RANDOM_2)
  {
    if(left){
      RCLCPP_INFO(LOGGER, "[Movement type random 1]");
      arm_system.arm_1.pose.position.z = 1.48 + cache_2;
    }

    if(right){
      arm_system.arm_2.pose.position.z = 1.48 + cache_2;
    }
  }

  if(executeOneOnly){

    bool executionSuccessful = false;
    int counter = 0;

    while (!executionSuccessful)
    {
      bool a_bot_found_ik = kinematic_state->setFromIK(arm_system.arm_1.arm_joint_model_group, arm_system.arm_1.pose, timeout);
      
      if (a_bot_found_ik)
      {
        kinematic_state->copyJointGroupPositions(arm_system.arm_1.arm_joint_model_group, arm_system.arm_1.arm_joint_values);
        dual_arm.setJointValueTarget(arm_system.arm_1.arm_joint_names, arm_system.arm_1.arm_joint_values);
        RCLCPP_INFO(LOGGER, "IK found for arm 1");
      }

      std::vector<double> arm_joint_values(arm_system.arm_2.arm_joint_names.size(), 0);
      dual_arm.setJointValueTarget(arm_system.arm_2.arm_joint_names, arm_joint_values );

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      if (a_bot_found_ik)
      {
        executionSuccessful = dual_arm.move() == moveit::core::MoveItErrorCode::SUCCESS;
        if(!executionSuccessful){
            objs.push(arm_system.arm_1.object);
            pnp_1->open_gripper();
            return false;
          }
      }
      else
      {
        if (movement == Movement::PREGRASP || movement == Movement::GRASP)
        {
          objs.push(arm_system.arm_1.object);
          return false;
        }
      }
    }
    
  }
  else
  {
    bool executionSuccessful = false;
    int counter = 0;
    while (!executionSuccessful)
    {

      bool a_bot_found_ik = kinematic_state->setFromIK(arm_system.arm_1.arm_joint_model_group, arm_system.arm_1.pose, timeout);
      bool b_bot_found_ik = kinematic_state->setFromIK(arm_system.arm_2.arm_joint_model_group, arm_system.arm_2.pose, timeout);

      if (a_bot_found_ik)
      {
        kinematic_state->copyJointGroupPositions(arm_system.arm_1.arm_joint_model_group, arm_system.arm_1.arm_joint_values);
        dual_arm.setJointValueTarget(arm_system.arm_1.arm_joint_names, arm_system.arm_1.arm_joint_values);
        RCLCPP_INFO(LOGGER, "IK found for arm 1");
      }

      if (b_bot_found_ik)
      {
        kinematic_state->copyJointGroupPositions(arm_system.arm_2.arm_joint_model_group, arm_system.arm_2.arm_joint_values);
        dual_arm.setJointValueTarget(arm_system.arm_2.arm_joint_names, arm_system.arm_2.arm_joint_values);
        RCLCPP_INFO(LOGGER, "IK found for arm 2");
      }

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      if (a_bot_found_ik && b_bot_found_ik)
      {
        executionSuccessful = dual_arm.move() == moveit::core::MoveItErrorCode::SUCCESS;
        if(!executionSuccessful){
            objs.push(arm_system.arm_1.object);
            objs.push(arm_system.arm_2.object);
            pnp_1->open_gripper();
            pnp_2->open_gripper();
            return false;
          }
      }
      else
      {
        if (movement == Movement::PREGRASP || movement == Movement::GRASP)
        {
          objs.push(arm_system.arm_1.object);
          objs.push(arm_system.arm_2.object);
          return false;
        }
      }
    }
  }

  return true;
}