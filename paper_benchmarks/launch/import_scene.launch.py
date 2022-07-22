from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    scene_argument = PathJoinSubstitution(
        [get_package_share_directory("panda_description"), "scene", "trays.scene"]
    )

    print([get_package_share_directory("panda_description"), "scene", "trays.scene"])

    # Start the actual move_group node/action server
    import_scene = Node(
        package="moveit_ros_planning",
        executable="moveit_publish_scene_from_text",
        output="screen",
        arguments=[
            "--scene",scene_argument,
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(import_scene)    

    return ld   