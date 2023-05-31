#ifndef BENCHMARK_SYNCHRONOUS_H
#define BENCHMARK_SYNCHRONOUS_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "paper_benchmarks/scene.hpp"
#include "paper_benchmarks/primitive_pick_and_place.hpp"
#include "std_msgs/msg/string.hpp"
#include "paper_benchmarks/cube_selector.hpp"

rclcpp::Node::SharedPtr node;

std::shared_ptr<primitive_pick_and_place> pnp_1;
std::shared_ptr<primitive_pick_and_place> pnp_2;
std::shared_ptr<primitive_pick_and_place> pnp_dual;

//moveit::planning_interface::MoveGroupInterface arm_group("dual_arm");

tray_helper blue_tray_1(4,4,0.11,-0.925,0.06,0.1,true);
tray_helper red_tray_1(4,4,-0.425,-0.925,0.06,0.1,true);
tray_helper blue_tray_2(4,4,0.11,0.925,0.06,0.1,false);
tray_helper red_tray_2(4,4,-0.425,0.925,0.06,0.1,false);

const rclcpp::Logger LOGGER = rclcpp::get_logger("benchmark_synchronous");
void main_thread();
void update_planning_scene();

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
std::vector<std::string> all_objects;
Point3D e(0,0,0);
ThreadSafeCubeQueue objs(e);

struct arm_state{
    const moveit::core::JointModelGroup* arm_joint_model_group;
    const std::vector<std::string> &arm_joint_names;
    std::vector<double> arm_joint_values;
    geometry_msgs::msg::Pose pose; 
    CollisionPlanningObject object;

    arm_state(const moveit::core::JointModelGroup* jmg):
    arm_joint_model_group(jmg), arm_joint_names(jmg->getVariableNames()){}
};

enum Movement{
    PREGRASP,
    GRASP,
    PREMOVE,
    MOVE,
    PUTDOWN,
    POSTMOVE
};

struct dual_arm_state{
    arm_state arm_1;
    arm_state arm_2;

    dual_arm_state(arm_state a_1, arm_state a_2): arm_1(a_1),arm_2(a_2) {}
};

std::map<std::string, moveit_msgs::msg::CollisionObject> objMap;
std::map<std::string, moveit_msgs::msg::ObjectColor> colors;
bool update_scene_called_once = false;

bool plan_and_move(dual_arm_state &arm_system, Movement movement, moveit::core::RobotStatePtr kinematic_state, 
double timeout, moveit::planning_interface::MoveGroupInterface& dual_arm, tray_helper* active_tray_arm_1,
tray_helper* active_tray_arm_2);

#endif