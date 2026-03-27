from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# All 25 joints in HARDWARE_JOINT_ORDER
ALL_JOINTS = [
    "left_shoulder_pitch_joint_X6", "left_shoulder_roll_joint_X6",
    "left_shoulder_yaw_joint_X4", "left_elbow_pitch_joint_X6",
    "left_wrist_yaw_joint_X4", "left_wrist_roll_joint_X4",
    "right_shoulder_pitch_joint_X6", "right_shoulder_roll_joint_X6",
    "right_shoulder_yaw_joint_X4", "right_elbow_pitch_joint_X6",
    "right_wrist_yaw_joint_X4", "right_wrist_roll_joint_X4",
    "waist_yaw_joint_X8",
    "left_hip_pitch_joint_X8", "left_hip_roll_joint_X8",
    "left_hip_yaw_joint_X8", "left_knee_joint_X8",
    "left_ankle_pitch_joint_X4", "left_ankle_roll_joint_X4",
    "right_hip_pitch_joint_X8", "right_hip_roll_joint_X8",
    "right_hip_yaw_joint_X8", "right_knee_joint_X8",
    "right_ankle_pitch_joint_X4", "right_ankle_roll_joint_X4",
]


def generate_launch_description():
    pkg_full_robot_description = FindPackageShare("full_robot_description")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    control_mode_arg = DeclareLaunchArgument(
        "control_mode", default_value="position",
        description="Control mode: position, effort, or gz_policy"
    )
    onnx_path_arg = DeclareLaunchArgument(
        "onnx_path", default_value="",
        description="Path to ONNX policy model (for gz_policy mode)"
    )
    policy_debug_arg = DeclareLaunchArgument(
        "policy_debug", default_value="false",
        description="Enable policy debug topic publishing"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="1.25", description="Z position")

    use_sim_time = LaunchConfiguration("use_sim_time")
    control_mode = LaunchConfiguration("control_mode")
    onnx_path = LaunchConfiguration("onnx_path")
    policy_debug = LaunchConfiguration("policy_debug")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    is_gz_native = PythonExpression(["'", control_mode, "' == 'gz_policy'"])
    is_ros2_control = PythonExpression(["'", control_mode, "' != 'gz_policy'"])

    # Robot description from XACRO
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_full_robot_description, "urdf", "full_robot_gazebo.xacro"]),
        " control_mode:=", control_mode,
        " onnx_path:=", onnx_path,
        " policy_debug:=", policy_debug,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "harambe",
            "-topic", "robot_description",
            "-x", x_pos, "-y", y_pos, "-z", z_pos,
        ],
        output="screen",
    )

    # ========== ros2_control mode (position/effort/effort_implicit_actuators) ==========
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30",
            "--switch-timeout", "30",
            "--service-call-timeout", "60",
        ],
        output="screen",
        condition=IfCondition(is_ros2_control),
    )

    whole_body_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
        condition=IfCondition(is_ros2_control),
    )

    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_whole_body_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[whole_body_controller_spawner],
        )
    )

    # ========== gz_policy mode: bridge Gazebo joint states to ROS 2 ==========
    gz_joint_bridge_args = []
    for jname in ALL_JOINTS:
        # ROS 2 → Gazebo: position command per joint
        gz_joint_bridge_args.append(
            f"/joint_cmd/{jname}@std_msgs/msg/Float64]gz.msgs.Double"
        )
    # Gazebo → ROS 2: joint states
    gz_joint_bridge_args.append(
        "/joint_states_gz@sensor_msgs/msg/JointState[gz.msgs.Model"
    )

    gz_joint_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_joint_bridge",
        arguments=gz_joint_bridge_args,
        output="screen",
        condition=IfCondition(is_gz_native),
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg, control_mode_arg, onnx_path_arg, policy_debug_arg, x_arg, y_arg, z_arg,

        # RSP + spawn (always)
        robot_state_publisher,
        spawn_entity,

        # ros2_control chain (position/effort modes)
        start_jsb_after_spawn,
        start_whole_body_after_jsb,

        # gz_pid mode: Gazebo-native joint controllers + bridge
        gz_joint_bridge,
    ])
