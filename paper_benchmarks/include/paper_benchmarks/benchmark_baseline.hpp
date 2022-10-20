#ifndef BENCHMARK_BASELINE_H
#define BENCHMARK_BASELINE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include "paper_benchmarks/primitive_pick_and_place.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

rclcpp::Node::SharedPtr node;
std::shared_ptr<primitive_pick_and_place> pnp;
const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_baseline");
void main_thread();

#endif