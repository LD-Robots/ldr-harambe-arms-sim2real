"""
Launch dual-arm hand tracking teleop with Orbbec Gemini 336L + MoveIt Servo.

Sequence:
1. Publish static TF: camera_link -> urdf_base
2. Move both arms to ready position (via trajectory controllers)
3. Unload left_arm_controller + right_arm_controller (release command interfaces)
4. Spawn left_servo_controller + right_servo_controller (forward_command)
5. Start two MoveIt Servo nodes (left_arm + right_arm, POSE mode)
6. Start hand tracking teleop node

Prerequisites:
  - Full dual-arm system running (Gazebo + controllers)
  - Orbbec camera running:
      ros2 launch orbbec_camera gemini_330_series.launch.py depth_registration:=true device_preset:=Hand

Usage:
    ros2 launch arm_teleop servo_hand_tracking_teleop.launch.py
    ros2 launch arm_teleop servo_hand_tracking_teleop.launch.py debug_mode:=true
    ros2 launch arm_teleop servo_hand_tracking_teleop.launch.py camera_x:=0.8 camera_z:=0.4
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # Launch arguments
    # ------------------------------------------------------------------ #
    args = [
        DeclareLaunchArgument("debug_mode", default_value="false"),
        DeclareLaunchArgument("smoothing_alpha", default_value="0.4"),
        DeclareLaunchArgument("min_detection_confidence", default_value="0.7"),
        DeclareLaunchArgument("min_tracking_confidence", default_value="0.5"),
        DeclareLaunchArgument("hand_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        DeclareLaunchArgument("linear_scale", default_value="0.3"),
        DeclareLaunchArgument("angular_scale", default_value="0.5"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # Camera static TF (camera position relative to urdf_base)
        DeclareLaunchArgument("camera_x", default_value="1.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.5"),
        DeclareLaunchArgument("camera_roll", default_value="0.0"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="3.14159"),  # facing robot
    ]

    # ------------------------------------------------------------------ #
    # MoveIt config (dual arm)
    # ------------------------------------------------------------------ #
    moveit_config = (
        MoveItConfigsBuilder(
            "dual_arm_description", package_name="dual_arm_moveit_config"
        )
        .robot_description(
            file_path="config/dual_arm_description.urdf.xacro",
            mappings={"use_sim": "true"},
        )
        .robot_description_semantic(
            file_path="config/dual_arm_description.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------ #
    # Servo parameters (shared base, per-arm overrides)
    # ------------------------------------------------------------------ #
    def make_servo_params(side: str) -> dict:
        ee_frame = (
            "urdf_l_wrist_assembly" if side == "left"
            else "urdf_r_wrist_assembly"
        )
        return {
            "moveit_servo.move_group_name": f"{side}_arm",
            "moveit_servo.planning_frame": "urdf_base",
            "moveit_servo.ee_frame_name": ee_frame,
            "moveit_servo.robot_link_command_frame": "urdf_base",
            "moveit_servo.command_in_type": "speed_units",
            "moveit_servo.scale.linear": LaunchConfiguration("linear_scale"),
            "moveit_servo.scale.rotational": LaunchConfiguration("angular_scale"),
            "moveit_servo.publish_period": PythonExpression(
                ["1.0 / ", LaunchConfiguration("publish_rate")]
            ),
            "moveit_servo.low_latency_mode": False,
            "moveit_servo.publish_joint_positions": True,
            "moveit_servo.publish_joint_velocities": False,
            "moveit_servo.publish_joint_accelerations": False,
            "moveit_servo.cartesian_command_in_topic": "~/delta_twist_cmds",
            "moveit_servo.joint_command_in_topic": "~/delta_joint_cmds",
            "moveit_servo.pose_command_in_topic": "~/pose_target_cmds",
            "moveit_servo.command_out_topic": f"/{side}_servo_controller/commands",
            "moveit_servo.command_out_type": "std_msgs/Float64MultiArray",
            "moveit_servo.status_topic": "~/status",
            "moveit_servo.joint_topic": "/joint_states",
            "moveit_servo.check_collisions": False,
            "moveit_servo.collision_check_rate": 10.0,
            "moveit_servo.lower_singularity_threshold": 1e9,
            "moveit_servo.hard_stop_singularity_threshold": 1e10,
            "moveit_servo.leaving_singularity_threshold_multiplier": 1.0,
            "moveit_servo.joint_limit_margins": [0.01] * 6,
            "moveit_servo.incoming_command_timeout": 0.5,
            "moveit_servo.use_smoothing": False,
        }

    # ------------------------------------------------------------------ #
    # Step 0a: Launch Orbbec camera
    # ------------------------------------------------------------------ #
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    orbbec_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("orbbec_camera"),
                "launch",
                "gemini_330_series.launch.py",
            ])
        ]),
        launch_arguments={
            "depth_registration": "true",
            "device_preset": "High Accuracy",
            "enable_color": "true",
            "enable_depth": "true",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # Step 0b: Static TF (camera_link -> urdf_base)
    # ------------------------------------------------------------------ #
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_base_tf",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--roll", LaunchConfiguration("camera_roll"),
            "--pitch", LaunchConfiguration("camera_pitch"),
            "--yaw", LaunchConfiguration("camera_yaw"),
            "--frame-id", "urdf_base",
            "--child-frame-id", "camera_link",
        ],
    )

    # ------------------------------------------------------------------ #
    # Step 1: Move both arms to ready position
    # ------------------------------------------------------------------ #
    ready_positions = "0.1, 0.1, 0.1, -0.1, 0.1, 0.1"

    move_left_ready = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/left_arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            "{trajectory: {joint_names: ["
            "left_shoulder_pitch_joint_X6, left_shoulder_roll_joint_X6, "
            "left_shoulder_yaw_joint_X4, left_elbow_pitch_joint_X6, "
            "left_wrist_yaw_joint_X4, left_wrist_roll_joint_X4], "
            f"points: [{{positions: [{ready_positions}], "
            "time_from_start: {sec: 2, nanosec: 0}}]}}",
        ],
        output="screen",
    )

    move_right_ready = ExecuteProcess(
        cmd=[
            "ros2", "action", "send_goal",
            "/right_arm_controller/follow_joint_trajectory",
            "control_msgs/action/FollowJointTrajectory",
            "{trajectory: {joint_names: ["
            "right_shoulder_pitch_joint_X6, right_shoulder_roll_joint_X6, "
            "right_shoulder_yaw_joint_X4, right_elbow_pitch_joint_X6, "
            "right_wrist_yaw_joint_X4, right_wrist_roll_joint_X4], "
            f"points: [{{positions: [{ready_positions}], "
            "time_from_start: {sec: 2, nanosec: 0}}]}}",
        ],
        output="screen",
    )

    # ------------------------------------------------------------------ #
    # Step 2: Deactivate + unload arm trajectory controllers (t=4s, 5s)
    # ------------------------------------------------------------------ #
    deactivate_left = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="Step 2: Deactivating arm controllers..."),
            ExecuteProcess(
                cmd=["ros2", "control", "set_controller_state",
                     "left_arm_controller", "inactive"],
                output="screen",
            ),
        ],
    )
    deactivate_right = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "set_controller_state",
                     "right_arm_controller", "inactive"],
                output="screen",
            ),
        ],
    )
    unload_left = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "unload_controller",
                     "left_arm_controller"],
                output="screen",
            ),
        ],
    )
    unload_right = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "unload_controller",
                     "right_arm_controller"],
                output="screen",
            ),
        ],
    )

    # ------------------------------------------------------------------ #
    # Step 3: Spawn servo controllers (t=7s)
    # The controller_manager was started before our servo controller
    # definitions existed. We must:
    #   a) Set the controller type on controller_manager params
    #   b) Spawn with --param-file for controller-specific params
    # ------------------------------------------------------------------ #
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("dual_arm_control"), "config", "controllers.yaml"
    ])

    # 3a: Register controller types on the controller_manager
    set_left_type = TimerAction(
        period=6.5,
        actions=[
            LogInfo(msg="Step 3a: Registering servo controller types..."),
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/controller_manager",
                     "left_servo_controller.type",
                     "forward_command_controller/ForwardCommandController"],
                output="screen",
            ),
        ],
    )
    set_right_type = TimerAction(
        period=6.5,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/controller_manager",
                     "right_servo_controller.type",
                     "forward_command_controller/ForwardCommandController"],
                output="screen",
            ),
        ],
    )

    # 3b: Spawn the controllers
    spawn_left_servo = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Step 3b: Spawning servo controllers..."),
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner",
                     "left_servo_controller",
                     "--param-file", controllers_yaml],
                output="screen",
            ),
        ],
    )
    spawn_right_servo = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "run", "controller_manager", "spawner",
                     "right_servo_controller",
                     "--param-file", controllers_yaml],
                output="screen",
            ),
        ],
    )

    # ------------------------------------------------------------------ #
    # Step 4: Start MoveIt Servo nodes (t=12s)
    # ------------------------------------------------------------------ #
    left_servo_node = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="Step 4: Starting MoveIt Servo nodes..."),
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="left_servo_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    make_servo_params("left"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
    )

    right_servo_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="moveit_servo",
                executable="servo_node",
                name="right_servo_node",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                    make_servo_params("right"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
    )

    # ------------------------------------------------------------------ #
    # Step 5: Start hand tracking teleop (t=18s)
    # ------------------------------------------------------------------ #
    tracking_node = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="Step 5: Starting hand tracking teleop..."),
            Node(
                package="arm_teleop",
                executable="servo_hand_tracking_teleop.py",
                name="servo_hand_tracking_teleop",
                output="screen",
                parameters=[{
                    "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
                    "debug_mode": LaunchConfiguration("debug_mode"),
                    "min_detection_confidence": LaunchConfiguration(
                        "min_detection_confidence"
                    ),
                    "min_tracking_confidence": LaunchConfiguration(
                        "min_tracking_confidence"
                    ),
                    "hand_publish_rate": LaunchConfiguration(
                        "hand_publish_rate"
                    ),
                    "tracking_enabled": True,
                    "target_frame": "urdf_base",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
            ),
        ],
    )

    # ------------------------------------------------------------------ #
    # Assemble launch description
    # ------------------------------------------------------------------ #
    return LaunchDescription(
        args
        + [
            # Step 0a: Orbbec camera
            orbbec_launch,
            # Step 0b: Static TF
            static_tf,
            # Step 1: Move to ready
            LogInfo(msg="Step 1: Moving both arms to ready position..."),
            move_left_ready,
            move_right_ready,
            # Step 2: Deactivate + unload
            deactivate_left,
            deactivate_right,
            unload_left,
            unload_right,
            # Step 3a: Register controller types
            set_left_type,
            set_right_type,
            # Step 3b: Spawn servo controllers
            spawn_left_servo,
            spawn_right_servo,
            # Step 4: MoveIt Servo
            left_servo_node,
            right_servo_node,
            # Step 5: Tracking node
            tracking_node,
        ]
    )
