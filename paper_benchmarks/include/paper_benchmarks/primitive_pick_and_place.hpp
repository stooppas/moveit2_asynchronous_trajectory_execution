#ifndef PRIMITIVE_PICK_AND_PLACE_H
#define PRIMITIVE_PICK_AND_PLACE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include "paper_benchmarks/scene.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>

struct tray_helper{
    tray_helper(int x_limit,int y_limit,float x_offset,float y_offset,float x_spacing, float y_spacing,bool direction){
        this->x_limit = x_limit;
        this->y_limit = y_limit;
        this->x_offset = x_offset;
        this->y_offset = y_offset;
        this->x_spacing = x_spacing;
        this->y_spacing = y_spacing;
        this->direction = direction;
    }
    int x;
    int y; 
    int z;

    int x_limit;
    int y_limit; 

    float x_offset;
    float y_offset;

    float x_spacing;
    float y_spacing;
    bool direction;

    float get_x(){
        return x_offset + x * x_spacing;
    }

    float get_y(){
        if(direction){
            return y_offset + y * y_spacing;
        }else{
            return y_offset - y * y_spacing;
        }
    }


    void next(){
        if(x + 1 < x_limit){
            x++;
        }else{
            x = 0;
            if(y + 1 < y_limit){
                y++;
            }else{
                y = 0;
                z++;
            }
        }
    }
};

class primitive_pick_and_place{
    public: 
        primitive_pick_and_place(rclcpp::Node::SharedPtr node, std::string move_group,double timeout_duration = 60);
        bool set_joint_values_from_pose(geometry_msgs::msg::Pose& pose);
        std::vector<double> get_joint_values();
        bool generate_plan();
        bool execute();
        bool plan_and_execute();
        bool open_gripper();
        bool close_gripper();
        bool grasp_object(moveit_msgs::msg::CollisionObject& object);
        bool release_object(moveit_msgs::msg::CollisionObject& object);
        std::map<std::string, moveit_msgs::msg::CollisionObject> getCollisionObjects();
        std::map<std::string, moveit_msgs::msg::ObjectColor> getCollisionObjectColors();
        bool home();

    private:
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_interface;
        std::shared_ptr<moveit::core::RobotState> current_state;
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