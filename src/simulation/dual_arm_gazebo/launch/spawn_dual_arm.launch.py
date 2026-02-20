from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Packages
    pkg_dual_arm_description = FindPackageShare("dual_arm_description")

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="1.0", description="Z position")

    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pos = LaunchConfiguration("x")
    y_pos = LaunchConfiguration("y")
    z_pos = LaunchConfiguration("z")

    # Robot description from XACRO
    robot_description_content = Command([
        "xacro ",
        PathJoinSubstitution([pkg_dual_arm_description, "urdf", "dual_arm.urdf.xacro"]),
        " use_sim:=true",
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
            "-name", "dual_arm",
            "-topic", "robot_description",
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
        ],
        output="screen",
    )

    # Controller spawners - chained sequentially via OnProcessExit events.
    # Each spawner waits for the previous one to finish before starting.
    # Switch takes ~48s wall clock due to heavy sim, so sequential chaining
    # is more reliable than fixed TimerActions.
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_hand_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "20",
            "--switch-timeout", "20",
            "--service-call-timeout", "60",
        ],
        output="screen",
    )

    # Chain: spawn_entity → (5s delay) → JSB → left_arm → right_arm → left_hand → right_hand
    # Each spawner starts only after the previous one exits successfully.
    start_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[TimerAction(period=5.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    start_left_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[left_arm_controller_spawner],
        )
    )

    start_right_arm_after_left_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_arm_controller_spawner,
            on_exit=[right_arm_controller_spawner],
        )
    )

    start_left_hand_after_right_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=right_arm_controller_spawner,
            on_exit=[left_hand_controller_spawner],
        )
    )

    start_right_hand_after_left_hand = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=left_hand_controller_spawner,
            on_exit=[right_hand_controller_spawner],
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
        use_sim_time_arg, x_arg, y_arg, z_arg,

        # RSP first, then spawn entity
        robot_state_publisher,
        spawn_entity,

        # Sequential controller chain (triggered by events)
        start_jsb_after_spawn,
        start_left_arm_after_jsb,
        start_right_arm_after_left_arm,
        start_left_hand_after_right_arm,
        start_right_hand_after_left_hand,

        # Camera
        camera_bridge,
        camera_frame_fix,
    ])
