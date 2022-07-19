#ifndef PRIMITIVE_PICK_AND_PLACE_H
#define PRIMITIVE_PICK_AND_PLACE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class primitive_pick_and_place{
    public: 
        primitive_pick_and_place(rclcpp::Node::SharedPtr node, std::string move_group,double timeout_duration = 60);
        bool generate_pre_grasp_pose();
        bool generate_grasp_pose();
        bool generate_post_grasp_pose();
        bool generate_move_pose();
        bool generate_place_pose();
        bool generate_post_place_pose();
        std::vector<double> get_joint_values();
        bool generate_plan();
        bool execute();
        bool plan_and_execute();
        bool open_gripper();
        bool close_gripper();
        void set_active_object(moveit_msgs::msg::CollisionObject& object);
        bool grasp_object();
        bool release_object();

    private:
        bool pose_to_joint_values(geometry_msgs::msg::Pose& pose);
        std::string move_group;
        rclcpp::Node::SharedPtr node;
        moveit::core::RobotModelConstPtr robot_model;
        const moveit::core::JointModelGroup* joint_model_group;
        std::vector<std::string> joint_names;
        std::vector<double> joint_values;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_interface;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        double timeout_duration;
        bool has_gripper = false;
        int counter = 0;
        moveit_msgs::msg::CollisionObject active_object;

};

#endif