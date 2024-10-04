from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import *



def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("low_cost_robot", package_name="low_cost_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)