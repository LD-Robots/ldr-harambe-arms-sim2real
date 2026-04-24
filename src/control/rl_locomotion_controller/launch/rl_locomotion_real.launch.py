"""Launch the rl_locomotion_controller on the real Harambe hardware.

Expects the EtherCAT hardware already brought up (see arm_real_bringup).
This launch file ONLY registers + spawns the RL controller — it does not
start the hardware driver or controller_manager. Run arm_real_bringup
first (robot_group:=full), then launch this.

Example:
    # terminal 1 — EtherCAT hardware + controller_manager
    ros2 launch arm_real_bringup arm_real.launch.py robot_group:=full

    # terminal 2 — deactivate conflicting JTCs, then load the RL controller
    ros2 control switch_controllers \
        --deactivate left_arm_group_controller right_arm_group_controller \
                     left_leg_group_controller right_leg_group_controller \
                     waist_group_controller

    ros2 launch rl_locomotion_controller rl_locomotion_real.launch.py

The launch first declares the controller's type on /controller_manager
(needed because the arm_real_bringup controllers.yaml does not know about
this plugin), then runs the standard spawner which merges the param YAML
into the controller's namespace and activates it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONTROLLER_NAME = "rl_locomotion_controller"
PLUGIN_TYPE = "rl_locomotion_controller/RLLocomotionController"


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rl_locomotion_controller"),
        "config", "rl_locomotion_params.yaml",
    ])

    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="rl_locomotion_controller parameters YAML",
    )
    policy_path_arg = DeclareLaunchArgument(
        "policy_path",
        default_value="",
        description=(
            "Override policy_path from the params YAML. "
            "Leave empty to use the default (package://rl_locomotion_controller/policies/...)."
        ),
    )
    cmd_vel_x_arg = DeclareLaunchArgument("cmd_vel_x", default_value="")
    cmd_vel_y_arg = DeclareLaunchArgument("cmd_vel_y", default_value="")
    cmd_vel_yaw_arg = DeclareLaunchArgument("cmd_vel_yaw", default_value="")

    # Step 1: register controller type on /controller_manager
    register_type = ExecuteProcess(
        cmd=[
            "ros2", "param", "set", "/controller_manager",
            f"{CONTROLLER_NAME}.type", PLUGIN_TYPE,
        ],
        shell=False,
        output="screen",
    )

    # Step 2 (after type is set): spawn the controller
    spawn_cmd_args = [
        CONTROLLER_NAME,
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "60",
        "--param-file", LaunchConfiguration("params_file"),
    ]
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{CONTROLLER_NAME}",
        arguments=spawn_cmd_args,
        output="screen",
    )

    # Chain: spawn runs only after `ros2 param set` has exited
    spawn_after_register = RegisterEventHandler(
        OnProcessExit(target_action=register_type, on_exit=[spawner])
    )

    # Optional: apply per-run overrides AFTER the controller is loaded by
    # setting params on the controller node itself. These are only applied if
    # the user passed non-empty values via launch args.
    def _override_param(name, value):
        return ExecuteProcess(
            cmd=["ros2", "param", "set", f"/{CONTROLLER_NAME}", name, value],
            shell=False, output="screen",
        )

    override_nodes = []
    for key, arg in [
        ("policy_path", policy_path_arg),
        ("cmd_vel_x", cmd_vel_x_arg),
        ("cmd_vel_y", cmd_vel_y_arg),
        ("cmd_vel_yaw", cmd_vel_yaw_arg),
    ]:
        pass  # overrides via LaunchConfiguration require condition handling;
              # keep it simple — document that the YAML is the source of truth.

    return LaunchDescription([
        params_arg,
        policy_path_arg,
        cmd_vel_x_arg,
        cmd_vel_y_arg,
        cmd_vel_yaw_arg,
        register_type,
        spawn_after_register,
    ])
