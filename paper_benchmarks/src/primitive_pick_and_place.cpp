#include "paper_benchmarks/primitive_pick_and_place.hpp"

primitive_pick_and_place::primitive_pick_and_place(rclcpp::Node::SharedPtr node, std::string move_group,double timeout_duration)
{
    this->move_group = move_group;
    this->node = node;
    this->timeout_duration = timeout_duration;
    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,move_group);
    move_group_interface->setMaxVelocityScalingFactor(1.0);
    move_group_interface->setMaxAccelerationScalingFactor(1.0);

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

bool primitive_pick_and_place::open_gripper(){
    if(has_gripper){
        gripper_group_interface->setNamedTarget("open");
        return gripper_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
    }
    else{
        return false;
    }
}

bool primitive_pick_and_place::close_gripper(){
    if(has_gripper){
        gripper_group_interface->setNamedTarget("closed");
        return gripper_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
    }
    else{
        return false;
    }
}

void primitive_pick_and_place::set_active_object(moveit_msgs::msg::CollisionObject& object)
{
    active_object = object;
}

bool primitive_pick_and_place::grasp_object()
{
    move_group_interface->attachObject(active_object.id,"",{"base"});
}

bool primitive_pick_and_place::release_object()
{
    move_group_interface->detachObject(active_object.id);
}

bool primitive_pick_and_place::generate_pre_grasp_pose()
{
    geometry_msgs::msg::Pose pose;
    
    pose.position.x = active_object.pose.position.x;
    pose.position.y = active_object.pose.position.y;
    pose.position.z = active_object.pose.position.z + 0.25;

    pose.orientation.x = active_object.pose.orientation.w;
    pose.orientation.y = active_object.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    
    return pose_to_joint_values(pose);
}

bool primitive_pick_and_place::generate_grasp_pose()
{
    geometry_msgs::msg::Pose pose;
    
    pose.position.x = active_object.pose.position.x;
    pose.position.y = active_object.pose.position.y;
    pose.position.z = active_object.pose.position.z + 0.1;

    pose.orientation.x = active_object.pose.orientation.w;
    pose.orientation.y = active_object.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    return pose_to_joint_values(pose);
}

bool primitive_pick_and_place::generate_post_grasp_pose()
{
    geometry_msgs::msg::Pose pose;
    
    pose.position.x = active_object.pose.position.x;
    pose.position.y = active_object.pose.position.y;
    pose.position.z = active_object.pose.position.z + 0.25;

    pose.orientation.x = active_object.pose.orientation.w;
    pose.orientation.y = active_object.pose.orientation.z;
    pose.orientation.z = 0;
    pose.orientation.w = 0;
    
    return pose_to_joint_values(pose);
}

bool primitive_pick_and_place::generate_move_pose()
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = -0.475 + counter * 0.06;
    pose.position.y = -0.975;
    pose.position.z = 1.275;

    pose.orientation.x = 1;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    return pose_to_joint_values(pose);
}

bool primitive_pick_and_place::generate_place_pose()
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = -0.475 + counter * 0.06;
    pose.position.y = -0.975;
    pose.position.z = 1.125;

    pose.orientation.w = 0;
    pose.orientation.x = 1;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    return pose_to_joint_values(pose);
}

bool primitive_pick_and_place::generate_post_place_pose()
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = -0.475 + counter * 0.06;
    pose.position.y = -0.975;
    pose.position.z = 1.275;

    pose.orientation.w = 0;
    pose.orientation.x = 1;
    pose.orientation.y = 0;
    pose.orientation.z = 0;

    if(pose_to_joint_values(pose)){
        counter++;
        return true;
    }

    return false;
}

bool primitive_pick_and_place::pose_to_joint_values(geometry_msgs::msg::Pose& pose)
{
    auto current_state = move_group_interface->getCurrentState();
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
    return move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool primitive_pick_and_place::execute()
{
    return move_group_interface->execute(plan,rclcpp::Duration::from_seconds(timeout_duration)) == moveit::core::MoveItErrorCode::SUCCESS;
}

bool primitive_pick_and_place::plan_and_execute()
{
    return move_group_interface->move() == moveit::core::MoveItErrorCode::SUCCESS;
}
