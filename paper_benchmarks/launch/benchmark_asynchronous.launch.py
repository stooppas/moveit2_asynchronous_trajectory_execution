from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="paper_benchmarks",
        executable="benchmark_asynchronous",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    start_scene = IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/create_scene.launch.py"]))

    create_scene = IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/import_scene.launch.py"]))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(create_scene)
    ld.add_action(start_scene)   
    ld.add_action(move_group_node)

    return ld   