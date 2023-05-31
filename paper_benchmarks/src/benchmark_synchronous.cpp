#include "paper_benchmarks/benchmark_synchronous.hpp"
#include "paper_benchmarks/cube_selector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

bool move_(std::vector<double> arm_joint_values,
  const std::vector<std::string>& bot_joint_names, 
  moveit::core::RobotModelConstPtr kinematic_model, 
  const moveit::core::JointModelGroup* arm_joint_model_group,
  moveit::core::RobotStatePtr kinematic_state, double timeout,
  geometry_msgs::msg::Pose pose, CollisionPlanningObject object,
  moveit::planning_interface::MoveGroupInterface &dual_arm);

int main(int argc, char** argv)
{
  rclcpp::init(argc,argv);

  node = std::make_shared<rclcpp::Node>("benchmark_baseline");

  pnp_1 = std::make_shared<primitive_pick_and_place>(node,"panda_1");
  pnp_2 = std::make_shared<primitive_pick_and_place>(node,"panda_2");
  pnp_dual = std::make_shared<primitive_pick_and_place>(node,"dual_arm");


  // publisher_ = node->create_publisher<std_msgs::msg::String>("spawnNewCube", 10);

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
        objs.push( new_object);

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

  dual_arm.setMaxVelocityScalingFactor(1.0);
  dual_arm.setMaxAccelerationScalingFactor(1.0);
  dual_arm.setNumPlanningAttempts(5);
  dual_arm.setPlanningTime(1);

  RCLCPP_INFO(LOGGER, "[Initialized]");

  moveit::core::RobotModelConstPtr kinematic_model = panda_1_arm.getRobotModel();
  moveit::core::RobotStatePtr kinematic_state = dual_arm.getCurrentState();
  const moveit::core::JointModelGroup* arm_1_joint_model_group = kinematic_model->getJointModelGroup("panda_1");
  const moveit::core::JointModelGroup* arm_2_joint_model_group = kinematic_model->getJointModelGroup("panda_2");

  const std::vector<std::string>& a_bot_joint_names = arm_1_joint_model_group->getVariableNames();
  const std::vector<std::string>& b_bot_joint_names = arm_2_joint_model_group->getVariableNames();

  std::vector<double> arm_1_joint_values;
  std::vector<double> arm_2_joint_values;

  RCLCPP_INFO(LOGGER, "[Go to go]");


  srand(time(0));
  rclcpp::Rate r(1);
  bool success = false;

  while (!update_scene_called_once)
  {
    std::this_thread::sleep_for(1.0s);
  }

  RCLCPP_INFO(LOGGER,"Size: %li",objs.size());

  RCLCPP_INFO(LOGGER, "[checkpoint]");


  geometry_msgs::msg::Pose pose_1;
  geometry_msgs::msg::Pose pose_2;
  
  tray_helper blue_tray(6,3,0.11,-0.925,0.06,0.1,true);
  tray_helper red_tray(6,3,-0.425,-0.925,0.06,0.1,true);
  tray_helper* active_tray_arm_1;
  tray_helper* active_tray_arm_2;

  
  RCLCPP_INFO(LOGGER,"Finished");

  while(!objs.empty())
  {

    RCLCPP_INFO(LOGGER, "[starting pick and place]");

    CollisionPlanningObject object_1;
    CollisionPlanningObject object_2;

    object_1 = objs.pop("", "random" );
    object_2 = objs.pop("", "random" );

    RCLCPP_INFO(LOGGER, "[popped two]");
    RCLCPP_INFO(LOGGER, "[object id %s ]", object_1.collisionObject.id.c_str());
    RCLCPP_INFO(LOGGER, "[object id %s ]", object_2.collisionObject.id.c_str());

     RCLCPP_INFO(LOGGER,"Next tray selection");

    auto object_1_id  = object_1.collisionObject.id;
    auto object_2_id  = object_2.collisionObject.id;

    RCLCPP_INFO(LOGGER,"Size colors %li", colors.size());

    if (colors[object_1_id].color.r == 1 && colors[object_1_id].color.g == 0 && colors[object_1_id].color.b == 0)
      active_tray_arm_1 = &red_tray_1;
    else if (colors[object_1_id].color.r == 0 && colors[object_1_id].color.g == 0 && colors[object_1_id].color.b == 1)
      active_tray_arm_1 = &blue_tray_1;
    else
      continue;
    RCLCPP_INFO(LOGGER,"One done");

    if (colors[object_2_id].color.r == 1 && colors[object_2_id].color.g == 0 && colors[object_2_id].color.b == 0)
      active_tray_arm_2 = &red_tray_2;
    else if (colors[object_2_id].color.r == 0 && colors[object_2_id].color.g == 0 && colors[object_2_id].color.b == 1)
      active_tray_arm_2 = &blue_tray_2;
    else
      continue;
    RCLCPP_INFO(LOGGER,"Two done");
    

    //Pre Grasp
    pose_1.position.x = object_1.collisionObject.pose.position.x;
    pose_1.position.y = object_1.collisionObject.pose.position.y;
    pose_1.position.z = object_1.collisionObject.pose.position.z + 0.25;

    pose_1.orientation.x = object_1.collisionObject.pose.orientation.w;
    pose_1.orientation.y = object_1.collisionObject.pose.orientation.z;
    pose_1.orientation.z = 0;
    pose_1.orientation.w = 0;

    pose_2.position.x = object_2.collisionObject.pose.position.x;
    pose_2.position.y = object_2.collisionObject.pose.position.y;
    pose_2.position.z = object_2.collisionObject.pose.position.z + 0.25;

    pose_2.orientation.x = object_2.collisionObject.pose.orientation.w;
    pose_2.orientation.y = object_2.collisionObject.pose.orientation.z;
    pose_2.orientation.z = 0;
    pose_2.orientation.w = 0;
    RCLCPP_INFO(LOGGER, "[posed]");

    double timeout = 1;
    bool a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
    kinematic_state, timeout, pose_1, object_1, dual_arm);
    bool b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
    kinematic_state, timeout, pose_2, object_2, dual_arm);
    
    RCLCPP_INFO(LOGGER, "[pregrasps setting successful]");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if(a_bot_found_ik && b_bot_found_ik){
      bool success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(!success || my_plan.trajectory_.joint_trajectory.points.size() == 0){
        RCLCPP_INFO(LOGGER,"Plan did not succeed");
        objs.push(object_1);
        objs.push(object_2);
        continue;
      }

      dual_arm.execute(my_plan);
    }else{
      objs.push(object_1);
      objs.push(object_2);
      continue;

    }

    //grasp
    pose_1.position.z = object_1.collisionObject.pose.position.z + 0.1;
    pose_2.position.z = object_2.collisionObject.pose.position.z + 0.1;

    a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
    kinematic_state, timeout, pose_1, object_1, dual_arm);
    b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
    kinematic_state, timeout, pose_2, object_2, dual_arm);
    
    RCLCPP_INFO(LOGGER, "[grasps setting successful]");

    if(a_bot_found_ik && b_bot_found_ik){
      success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if(!success || my_plan.trajectory_.joint_trajectory.points.size() == 0){
        RCLCPP_INFO(LOGGER,"grasp Plan did not succeed");
        objs.push(object_1);
        objs.push(object_2);
        continue;
      }

      dual_arm.execute(my_plan);
      rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    }else{
      objs.push(object_1);
      objs.push(object_2);
      continue;
    }

    //pnp_1->close_gripper();
    //pnp_2->close_gripper();

    pnp_1->grasp_object(object_1.collisionObject);
    pnp_2->grasp_object(object_2.collisionObject);

    RCLCPP_INFO(LOGGER,"Next premove");
    //Premove
    pose_1.position.z = object_1.collisionObject.pose.position.z + 0.25;
    pose_2.position.z = object_2.collisionObject.pose.position.z + 0.25;

    // a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
    // kinematic_state, timeout, pose_1, object_1, dual_arm);
    // b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
    // kinematic_state, timeout, pose_2, object_2, dual_arm);
    
    // RCLCPP_INFO(LOGGER, "[premove setting successful]");

    // if(a_bot_found_ik && b_bot_found_ik){
    //   success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //   if(!success || my_plan.trajectory_.joint_trajectory.points.size() == 0){
    //     RCLCPP_INFO(LOGGER,"premove Plan did not succeed");
    //     //objs.push(object_1);
    //     //objs.push(object_2);
    //     continue;
    //   }

    //   dual_arm.execute(my_plan);
    //   rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    // }else{
    //   //objs.push(object_1);
    //   //objs.push(object_2);
    //   continue;
    // }

    success = false;
    while(!success){
      a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
      kinematic_state, timeout, pose_1, object_1, dual_arm);
      b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
      kinematic_state, timeout, pose_2, object_2, dual_arm);

      if(a_bot_found_ik && b_bot_found_ik){
        success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
          RCLCPP_INFO(LOGGER,"move Plan did not succeed hence planning again");
        }
      }
    }
    dual_arm.execute(my_plan);



    pose_1.position.x = active_tray_arm_1->get_x();
    pose_1.position.y = active_tray_arm_1->get_y();
    pose_1.position.z = 1.28 + active_tray_arm_1->z * 0.05;
    auto cache_1 =  active_tray_arm_1->z * 0.05;
    pose_1.orientation.x = 1;
    pose_1.orientation.y = 0;

    active_tray_arm_1->next();

    pose_2.position.x = active_tray_arm_2->get_x();
    pose_2.position.y = active_tray_arm_2->get_y();
    pose_2.position.z = 1.28 + active_tray_arm_2->z * 0.05;
    auto cache_2 =  active_tray_arm_2->z * 0.05;

    pose_2.orientation.x = 1;
    pose_2.orientation.y = 0;

    active_tray_arm_2->next();

    RCLCPP_INFO(LOGGER,"Next Move");
    

    //Move
    // a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
    // kinematic_state, timeout, pose_1, object_1, dual_arm);
    // b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
    // kinematic_state, timeout, pose_2, object_2, dual_arm);
    
    // RCLCPP_INFO(LOGGER, "[premove setting successful]");

    // if(a_bot_found_ik && b_bot_found_ik){
    //   success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //   if(!success || my_plan.trajectory_.joint_trajectory.points.size() == 0){
    //     RCLCPP_INFO(LOGGER,"premove Plan did not succeed");
    //     //objs.push(object_1);
    //     //objs.push(object_2);
    //     continue;
    //   }

    //   dual_arm.execute(my_plan);
    //   rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    // }else{
    //   //objs.push(object_1);
    //   //objs.push(object_2);
    //   continue;
    // }


    success = false;
    while(!success){
      a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
      kinematic_state, timeout, pose_1, object_1, dual_arm);
      b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
      kinematic_state, timeout, pose_2, object_2, dual_arm);

      if(a_bot_found_ik && b_bot_found_ik){
        success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
          RCLCPP_INFO(LOGGER,"move Plan did not succeed hence planning again");
        }
      }
    }
    dual_arm.execute(my_plan);

    RCLCPP_INFO(LOGGER,"Next putdown");
    
    //put down 
    pose_1.position.z = 1.141 + cache_1;
    pose_2.position.z = 1.141 + cache_2;

    // a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
    // kinematic_state, timeout, pose_1, object_1, dual_arm);
    // b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
    // kinematic_state, timeout, pose_2, object_2, dual_arm);
    
    //RCLCPP_INFO(LOGGER, "[premove setting successful]");

    // if(a_bot_found_ik && b_bot_found_ik){
    //   success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    //   if(!success || my_plan.trajectory_.joint_trajectory.points.size() == 0){
    //     RCLCPP_INFO(LOGGER,"premove Plan did not succeed");
    //     //objs.push(object_1);
    //     //objs.push(object_2);
    //     continue;
    //   }

    //   dual_arm.execute(my_plan);
    //   rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5)));
    // }else{
    //   //objs.push(object_1);
    //   //objs.push(object_2);
    //   continue;
    // }
    success = false;
    while(!success){
      a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
      kinematic_state, timeout, pose_1, object_1, dual_arm);
      b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
      kinematic_state, timeout, pose_2, object_2, dual_arm);

      if(a_bot_found_ik && b_bot_found_ik){
        success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
          RCLCPP_INFO(LOGGER,"put down Plan did not succeed hence planning again");
        }
      }
    }
    dual_arm.execute(my_plan);

    RCLCPP_INFO(LOGGER,"Next post move");

    pnp_1->release_object(object_1.collisionObject);
    pnp_2->release_object(object_2.collisionObject);

    pose_1.position.z = 1.28 + cache_1;
    pose_2.position.z = 1.28 + cache_2;

    //post move
    success = false;
    while(!success){
      a_bot_found_ik = move_(arm_1_joint_values, a_bot_joint_names, kinematic_model, arm_1_joint_model_group, 
      kinematic_state, timeout, pose_1, object_1, dual_arm);
      b_bot_found_ik = move_(arm_2_joint_values, b_bot_joint_names, kinematic_model, arm_2_joint_model_group, 
      kinematic_state, timeout, pose_2, object_2, dual_arm);

      if(a_bot_found_ik && b_bot_found_ik){
        success = (dual_arm.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(!success){
          RCLCPP_INFO(LOGGER,"post move Plan did not succeed hence planning again");
        }
      }
    }
    dual_arm.execute(my_plan);
  }

}
  //return true;

bool move_(std::vector<double> arm_joint_values,
  const std::vector<std::string>& bot_joint_names, 
  moveit::core::RobotModelConstPtr kinematic_model, 
  const moveit::core::JointModelGroup* arm_joint_model_group,
  moveit::core::RobotStatePtr kinematic_state, double timeout,
  geometry_msgs::msg::Pose pose, CollisionPlanningObject object,
  moveit::planning_interface::MoveGroupInterface &dual_arm)
  {
    bool a_bot_found_ik = kinematic_state->setFromIK(arm_joint_model_group, pose, timeout);
    RCLCPP_INFO(LOGGER, "[setting From IK successful]");

    if(!a_bot_found_ik)
    {
      RCLCPP_INFO(LOGGER,"Did not find IK solution for arm 1");
    }else{
      kinematic_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_values);
      dual_arm.setJointValueTarget(bot_joint_names, arm_joint_values);
    }
    
    return a_bot_found_ik;
}