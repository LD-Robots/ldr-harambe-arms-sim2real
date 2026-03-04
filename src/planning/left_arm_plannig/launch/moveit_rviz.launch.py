from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_harambe_v0.3", package_name="left_arm_plannig").to_moveit_configs()
    return generate_moveit_rviz_launch(moveit_config)
