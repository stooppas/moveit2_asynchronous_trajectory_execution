from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="paper_benchmarks",
        executable="benchmark_synchronous",
        output="screen",
        parameters=[],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(move_group_node)    

    return ld   