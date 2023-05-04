import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config_package = "panda_moveit_config"
    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .robot_description_semantic(file_path="config/panda.srdf")
        #.trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_full_config = PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Static TF
    #static_tf_node = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="static_transform_publisher",
    #    output="log",
    #    arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
    #)

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    panda_arm_1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_1_controller", "-c", "/controller_manager"],
    )

    panda_arm_2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_2_controller", "-c", "/controller_manager"],
    )

    panda_hand_1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_1_controller", "-c", "/controller_manager"],
    )

    panda_hand_2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_2_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            panda_arm_1_controller_spawner,
            panda_arm_2_controller_spawner,
            panda_hand_1_controller_spawner,
            panda_hand_2_controller_spawner
        ]
    )