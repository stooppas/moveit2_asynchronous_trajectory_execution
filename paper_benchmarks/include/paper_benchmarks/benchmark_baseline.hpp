#ifndef BENCHMARK_BASELINE_H
#define BENCHMARK_BASELINE_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"

rclcpp::Node::SharedPtr node;
std::shared_ptr<Scene> scene;
const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_baseline");
void main_thread();

#endif