"""
The full robot itself: robot_state_publisher, the Gazebo spawn, and the controllers.

Assumes Gazebo is already up -- full_robot_world.launch.py starts it and includes
this file. Run it on its own only against a simulation someone else launched.

The controller chain is sequenced on OnProcessExit rather than fixed timers: the
switch takes tens of seconds of wall clock under a heavy sim, and a timer long
enough to be safe is longer than the wait usually needs to be.

Usage:
    ros2 launch full_robot_gazebo spawn_full_robot.launch.py
    ros2 launch full_robot_gazebo spawn_full_robot.launch.py controller_type:=trajectory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_full_robot_description = FindPackageShare("full_robot_description")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    control_mode_arg = DeclareLaunchArgument(
        "control_mode", default_value="position",
        description="Control mode: position or effort"
    )
    controller_type_arg = DeclareLaunchArgument(
        "controller_type", default_value="direct",
        description="Controller type: 'direct' for JointGroupPositionController, "
                    "'trajectory' for JointTrajectoryController"
    )
    fixed_base_arg = DeclareLaunchArgument(
        "fixed_base", default_value="false",
        description="Fix robot base to world frame (no floating)"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.855", description="Z position")

    use_sim_time = LaunchConfiguration("use_sim_time")
    control_mode = LaunchConfiguration("control_mode")
    controller_type = LaunchConfiguration("controller_type")

    use_trajectory = PythonExpression(["'", controller_type, "' == 'trajectory'"])
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    fixed_base = LaunchConfiguration("fixed_base")

    # Robot description from XACRO (with control_mode and fixed_base args)
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_full_robot_description, "urdf", "full_robot_gazebo.xacro"]),
        " control_mode:=", control_mode,
        " fixed_base:=", fixed_base,
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
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
        ],
        output="screen",
    )

    # Controller spawners - chained sequentially via OnProcessExit events
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
    )

    spawner_args = [
        "--controller-manager", "/controller_manager",
        "--controller-manager-timeout", "20",
        "--switch-timeout", "20",
        "--service-call-timeout", "60",
    ]

    def _make_spawner(name, condition=None):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name] + spawner_args,
            output="screen",
            **({"condition": condition} if condition else {}),
        )

    # Direct position controller (JointGroupPositionController) — for RL policy deployment
    direct_controller_spawner = _make_spawner(
        "whole_body_controller", condition=UnlessCondition(use_trajectory),
    )

    # Per-group trajectory controllers — matches real hardware layout
    trajectory_controller_spawners = [
        _make_spawner(name, condition=IfCondition(use_trajectory))
        for name in [
            "left_arm_trajectory_controller",
            "right_arm_trajectory_controller",
            "waist_controller",
            "left_leg_trajectory_controller",
            "right_leg_trajectory_controller",
            # Hands disabled — inspire hands commented out in URDF
            # "left_hand_controller",
            # "right_hand_controller",
        ]
    ]

    # Chain: spawn_entity -> (5s) -> JSB -> controller (direct or trajectory)
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[direct_controller_spawner] + trajectory_controller_spawners,
        )
    )

    # Camera bridge - bridge Gazebo rgbd_camera topics to ROS 2
    camera_bridge = TimerAction(
        period=2.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/camera/depth_camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "--ros-args",
                "--param", "qos_overrides./camera/depth_image.publisher.depth:=1",
                "--param", "qos_overrides./camera/image.publisher.depth:=1",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            remappings=[
                ("/camera/image", "/camera/color/image_raw"),
                ("/camera/camera_info", "/camera/color/camera_info"),
                ("/camera/depth_image", "/camera/depth/image_raw"),
                ("/camera/depth_camera_info", "/camera/depth/camera_info"),
            ],
        )]
    )

    # Static transform to fix Gazebo's frame naming
    camera_frame_fix = TimerAction(
        period=3.0,
        actions=[Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "camera_link", "dual_arm/urdf_base/camera"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )]
    )
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg, control_mode_arg, controller_type_arg, fixed_base_arg,
        x_arg, y_arg, z_arg,

        # RSP first, then spawn entity
        robot_state_publisher,
        spawn_entity,

        # Sequential controller chain (triggered by events)
        start_jsb_after_spawn,
        start_controller_after_jsb,
        
        # Camera
        camera_bridge,
        camera_frame_fix,
    ])
