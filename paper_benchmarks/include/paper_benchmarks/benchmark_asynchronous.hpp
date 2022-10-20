#ifndef BENCHMARK_ASYNCHRONOUS_H
#define BENCHMARK_ASYNCHRONOUS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include "paper_benchmarks/primitive_pick_and_place.hpp"
#include <chrono>

using namespace std::chrono_literals;

rclcpp::Node::SharedPtr node;
std::shared_ptr<primitive_pick_and_place> pnp_1;
std::shared_ptr<primitive_pick_and_place> pnp_2;

tray_helper blue_tray_1(4,4,0.11,-0.925,0.06,0.1,true);
tray_helper red_tray_1(4,4,-0.425,-0.925,0.06,0.1,true);
tray_helper blue_tray_2(4,4,0.11,0.925,0.06,0.1,false);
tray_helper red_tray_2(4,4,-0.425,0.925,0.06,0.1,false);

bool panda_1_busy = false;
bool panda_2_busy = false;

const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_asynchronous");
void main_thread();
bool executeTrajectory(std::shared_ptr<primitive_pick_and_place> pnp,moveit_msgs::msg::CollisionObject& object,tray_helper* tray);

#endif