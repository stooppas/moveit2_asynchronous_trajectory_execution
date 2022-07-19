from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="paper_benchmarks",
        executable="benchmark_synchronous",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(move_group_node)    

    return ld   