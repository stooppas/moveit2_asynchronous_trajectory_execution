#include "paper_benchmarks/primitive_pick_and_place.hpp"

primitive_pick_and_place::primitive_pick_and_place(rclcpp::Node::SharedPtr node, std::string move_group,double timeout_duration)
{
    this->move_group = move_group;
    this->node = node;
    this->timeout_duration = timeout_duration;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,move_group);
    move_group_interface->setMaxVelocityScalingFactor(1.0);
    move_group_interface->setMaxAccelerationScalingFactor(1.0);
    move_group_interface->setNumPlanningAttempts(5);

    planning_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    robot_model = move_group_interface->getRobotModel();
    joint_model_group = robot_model->getJointModelGroup(move_group);
    joint_names = joint_model_group->getVariableNames();
    
    for(auto& eef : robot_model->getEndEffectors()){
        if(eef->getEndEffectorParentGroup().first == move_group){
            has_gripper = true;
            RCLCPP_INFO(rclcpp::get_logger("Primitive_Pick_And_Place"),"Registered Gripper");
            gripper_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,eef->getName());
        }
    }

}

bool primitive_pick_and_place::home()
{
    move_group_interface->setStartStateToCurrentState();
    move_group_interface->setNamedTarget("home");
    return plan_and_execute();
}

std::map<std::string, moveit_msgs::msg::CollisionObject> primitive_pick_and_place::getCollisionObjects(){
    return planning_interface->getObjects();
}

std::map<std::string, moveit_msgs::msg::ObjectColor> primitive_pick_and_place::getCollisionObjectColors(){
    return planning_interface->getObjectColors();
}


bool primitive_pick_and_place::open_gripper(){
    if(has_gripper){
        gripper_group_interface->setStartStateToCurrentState();
        gripper_group_interface->setNamedTarget("open");
        return gripper_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
    }
    else{
        return false;
    }
}

bool primitive_pick_and_place::close_gripper(){
    if(has_gripper){
        gripper_group_interface->setStartStateToCurrentState();
        gripper_group_interface->setNamedTarget("closed");
        return gripper_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
    }
    else{
        return false;
    }
}

bool primitive_pick_and_place::grasp_object(moveit_msgs::msg::CollisionObject& object)
{
    if(move_group == "panda_1") move_group_interface->attachObject(object.id,"",{"base","panda_1_leftfinger","panda_1_rightfinger","tray_red_1","tray_red_2","tray_blue_1","tray_blue_2",object.id});
    else if(move_group == "panda_2") move_group_interface->attachObject(object.id,"",{"base","panda_2_leftfinger","panda_2_rightfinger","tray_red_1","tray_red_2","tray_blue_1","tray_blue_2",object.id});
    primitive_pick_and_place::close_gripper();
}

bool primitive_pick_and_place::release_object(moveit_msgs::msg::CollisionObject& object)
{
    move_group_interface->detachObject(object.id);
    primitive_pick_and_place::open_gripper();
}

bool primitive_pick_and_place::set_joint_values_from_pose(geometry_msgs::msg::Pose& pose)
{
    current_state = move_group_interface->getCurrentState();
    bool found_ik = current_state->setFromIK(joint_model_group, pose, 0.1);

    if(!found_ik){
        RCLCPP_ERROR(node->get_logger(),"Did not find IK solution");
        return false;
    }

    current_state->copyJointGroupPositions(joint_model_group, joint_values);

    move_group_interface->setJointValueTarget(joint_names, joint_values);

    return true;
}

std::vector<double> primitive_pick_and_place::get_joint_values()
{
    return joint_values;
}

bool primitive_pick_and_place::generate_plan()
{
    move_group_interface->setStartStateToCurrentState();
    return move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool primitive_pick_and_place::execute()
{
    return move_group_interface->execute(plan,rclcpp::Duration::from_seconds(10)) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool primitive_pick_and_place::plan_and_execute()
{
    return move_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
}
