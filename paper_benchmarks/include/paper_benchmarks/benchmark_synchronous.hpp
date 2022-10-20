#ifndef BENCHMARK_SYNCHRONOUS_H
#define BENCHMARK_SYNCHRONOUS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include "paper_benchmarks/primitive_pick_and_place.hpp"

rclcpp::Node::SharedPtr node;

std::shared_ptr<primitive_pick_and_place> pnp_1;
std::shared_ptr<primitive_pick_and_place> pnp_2;
std::shared_ptr<primitive_pick_and_place> pnp_dual;

tray_helper blue_tray_1(4,4,0.11,-0.925,0.06,0.1,true);
tray_helper red_tray_1(4,4,-0.425,-0.925,0.06,0.1,true);
tray_helper blue_tray_2(4,4,0.11,0.925,0.06,0.1,false);
tray_helper red_tray_2(4,4,-0.425,0.925,0.06,0.1,false);

const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_synchronous");
void main_thread();

#endif