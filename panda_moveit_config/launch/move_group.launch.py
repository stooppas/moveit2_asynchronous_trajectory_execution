from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    ld = generate_move_group_launch(moveit_config)
    '''
    for entity in ld.entities:
        if type(entity) == Node:
            if entity.node_executable == "move_group":
                entity._Node__parameters[-1][(TextSubstitution(text="publish_planning_scene_hz"),)] = 30.0
    '''
    return ld