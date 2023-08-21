from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()

    background_r_launch_arg = DeclareLaunchArgument(
        "cubesToPick", default_value=TextSubstitution(text="5")
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="paper_benchmarks",
        executable="benchmark_baseline",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"cubesToPick" : LaunchConfiguration("cubesToPick")}
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
    ld.add_action(background_r_launch_arg)

    return ld   