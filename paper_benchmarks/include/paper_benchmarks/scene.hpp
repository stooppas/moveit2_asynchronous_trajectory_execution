#ifndef SETUP_SCENE_H
#define SETUP_SCENE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>

struct Block{
    Block(shape_msgs::msg::SolidPrimitive primitive,geometry_msgs::msg::Pose pose){
        this->pose = pose;
        this->primitive = primitive;
    }
    geometry_msgs::msg::Pose pose;
    shape_msgs::msg::SolidPrimitive primitive;
    std::string id;
};

class Scene{
    public:
        Scene(rclcpp::Node::SharedPtr node);
        bool attachObject();
        bool detachObject();

    private:
        rclcpp::Node::SharedPtr node;
        rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
        std::list<Block> blockList;

        void populate_blockList();
        void create_scene();
};
#endif