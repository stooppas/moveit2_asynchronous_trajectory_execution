#ifndef BENCHMARK_BASELINE_H
#define BENCHMARK_BASELINE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include "paper_benchmarks/primitive_pick_and_place.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

rclcpp::Node::SharedPtr node;
std::shared_ptr<primitive_pick_and_place> pnp;
const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_baseline");
void main_thread();
void update_planning_scene();

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
std::vector<std::string> all_objects;
Point3D e(0, 0, 0);
ThreadSafeCubeQueue objs(e);

std::map<std::string, moveit_msgs::msg::CollisionObject> objMap;
std::map<std::string, moveit_msgs::msg::ObjectColor> colors;
bool update_scene_called_once = false;

#endif