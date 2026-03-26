from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_harambe_v0.3", package_name="left_arm_plannig").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
