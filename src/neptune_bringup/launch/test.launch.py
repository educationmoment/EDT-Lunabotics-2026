import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
)


def generate_launch_description():
    config_dir       = get_package_share_directory("config_pkg")
    realsense_dir    = get_package_share_directory("realsense2_camera")

    nav2_params_file     = os.path.join(config_dir, "params", "nav2",               "nav2_uic_bot_params.yaml")
    rtabmap_params_file  = os.path.join(config_dir, "params", "rtabmap",             "rtabmap_params.yaml")
    ukf_params_file      = os.path.join(config_dir, "params", "robot_localization",  "ukf_params.yaml")
    s3_params_file       = os.path.join(config_dir, "params", "laser_filters",       "s3_params.yaml")
    apriltag_params_file = os.path.join(config_dir, "params", "apriltag",            "tag_params.yaml")
    bt_nav_to_pose       = os.path.join(config_dir, "behavior_trees", "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml")
    bt_nav_through_poses = os.path.join(config_dir, "behavior_trees", "nav_through_poses_w_replanning_and_recovery.xml")

    urdf_file = os.path.join(
        get_package_share_directory("description_pkg"), "urdf", "uic_bot.xacro"
    )
    robot_description = xacro.process_file(urdf_file, mappings={"use_sim": "false"}).toxml()

    declare_robot_mode = DeclareLaunchArgument(
        "robot_mode", default_value="manual", choices=["manual", "auto"]
    )

    # ── ROBOT STATE ───────────────────────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
            "ignore_timestamp": True,   # prevents stale joint state warnings
        }],
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher", executable="joint_state_publisher",
        output="screen", parameters=[{"use_sim_time": False}],
    )

    # ── LIDAR ─────────────────────────────────────────────────────────────────
    s3_lidar_node = Node(
        package="rplidar_ros", executable="rplidar_node", name="rplidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial", "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 1000000, "frame_id": "s3_lidar_link",
            "inverted": False, "scan_mode": "DenseBoost",
            "angle_compensate": True, "scan_frequency": 10.0,
        }],
        remappings=[("scan", "/scan_raw")],
    )
    s3_filter_node = Node(
        package="laser_filters", executable="scan_to_scan_filter_chain",
        parameters=[s3_params_file],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )

    # ── CAMERAS ───────────────────────────────────────────────────────────────
    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, "launch", "rs_launch.py")),
        launch_arguments={
            "camera_name": "d455", "camera_namespace": "", "device_type": "d455",
            "publish_tf": "true", "serial_no": "'318122303486'",
            "enable_gyro": "true", "enable_accel": "true", "unite_imu_method": "2",
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
        }.items(),
    )
    d456_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, "launch", "rs_launch.py")),
        launch_arguments={
            "camera_name": "d456", "camera_namespace": "", "device_type": "d455",
            "publish_tf": "true", "serial_no": "'308222300472'",
            "enable_gyro": "true", "enable_accel": "true", "unite_imu_method": "2",
            "depth_module.depth_profile": "640x480x30",
            "rgb_camera.color_profile": "640x480x30",
        }.items(),
    )


    # ── IMU ───────────────────────────────────────────────────────────────────
    imu_rotator_node = Node(package="util_pkg", executable="imu_rotator")
    d455_imu_filter = Node(
        package="imu_complementary_filter", executable="complementary_filter_node",
        name="d455_imu_filter", output="screen",
        parameters=[{
            "publish_tf": False, "fixed_frame": "odom",
            "do_bias_estimation": True, "do_adaptive_gain": True,
            "use_mag": False, "gain_acc": 0.01, "gain_mag": 0.01,
        }],
        remappings=[("imu/data_raw", "/d455/imu/data_raw"), ("imu/data", "/d455/imu/data")],
    )
    d456_imu_filter = Node(
        package="imu_complementary_filter", executable="complementary_filter_node",
        name="d456_imu_filter", output="screen",
        parameters=[{
            "publish_tf": False, "fixed_frame": "odom",
            "do_bias_estimation": True, "do_adaptive_gain": True,
            "use_mag": False, "gain_acc": 0.01, "gain_mag": 0.01,
        }],
        remappings=[("imu/data_raw", "/d456/imu/data_raw"), ("imu/data", "/d456/imu/data")],
    )

    # ── RGBD SYNC ─────────────────────────────────────────────────────────────
    rgbd_sync1_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync1",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[
            ("rgb/image", "/d456/color/image_raw"), ("depth/image", "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"), ("rgbd_image", "/d456/rgbd_image"),
        ],
        namespace="d456", arguments=["--ros-args", "--log-level", "error"],
    )
    rgbd_sync2_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync2",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[
            ("rgb/image", "/d455/color/image_raw"), ("depth/image", "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"), ("rgbd_image", "/d455/rgbd_image"),
        ],
        namespace="d455", arguments=["--ros-args", "--log-level", "error"],
    )

    # ── APRILTAG ──────────────────────────────────────────────────────────────
    apriltag_d455_node = Node(
        package="apriltag_ros", executable="apriltag_node", output="screen",
        parameters=[apriltag_params_file],
        remappings=[("/image_rect", "/d455/color/image_raw"), ("/camera_info", "/d455/color/camera_info")],
    )
    apriltag_d456_node = Node(
        package="apriltag_ros", executable="apriltag_node", output="screen",
        parameters=[apriltag_params_file],
        remappings=[("/image_rect", "/d456/color/image_raw"), ("/camera_info", "/d456/color/camera_info")],
    )

    # ── ODOMETRY: rf2o ONLY → UKF ─────────────────────────────────────────────
    # Single source. Never run icp or rgbd_odom at the same time — causes UKF jumps.
    rf2o_odometry_node = Node(
        package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry", output="screen",
        parameters=[{
            "laser_scan_topic": "/scan", "odom_topic": "/rf2o_odom", #maybe change to scan_raw? idk
            "publish_tf": False,                # UKF owns odom TF
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "laser_frame_id": "s3_lidar_link",  # real robot needs this
            "init_pose_from_topic": "",
            "freq": 10.0,
        }],
        arguments=["--ros-args", "--log-level", "error"],
    )

    ukf_node = Node(
        package="robot_localization", executable="ukf_node", name="ukf_filter_node",
        output="screen",
        parameters=[{"use_sim_time": False}, ukf_params_file],
    )

    # ── SLAM ──────────────────────────────────────────────────────────────────
    slam_node = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
        parameters=[
            rtabmap_params_file,
            {
                "use_sim_time": False,
                "rgbd_cameras": 2,
                "subscribe_depth": False, "subscribe_rgbd": True,
                "subscribe_rgb": False, "subscribe_odom_info": False,
                "odom_sensor_sync": True,
                "frame_id": "base_link", "map_frame_id": "map", "odom_frame_id": "odom",
                "odom_topic": "/odometry/filtered",
                "publish_tf": True,
                "publish_tf_odom": False,       # UKF owns odom→base_link
                "database_path": "",
                "approx_sync": True, 
                "approx_sync_max_interval": 0.1,
                "sync_queue_size": 1000,
                "topic_queue_size": 30,
                "subscribe_scan_cloud": False, "subscribe_scan": False,
                "wait_imu_to_init": False,      # real robot: IMU already stable at launch
                "imu_topic": "/d455/imu/data",
            },
        ],
        remappings=[
            ("rgbd_image0", "/d456/rgbd_image"),
            ("rgbd_image1", "/d455/rgbd_image"),
            #("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # ── NAV2 ──────────────────────────────────────────────────────────────────
    controller_server_node = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen", parameters=[nav2_params_file],
    )
    planner_server_node = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen", parameters=[nav2_params_file],
    )
    behavior_server_node = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen", parameters=[nav2_params_file],
    )
    bt_navigator_node = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=[nav2_params_file, {
            "default_nav_to_pose_bt_xml": bt_nav_to_pose,
            "default_nav_through_poses_bt_xml": bt_nav_through_poses,
        }],
    )
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[
            {"autostart": True}, {"node_timeout": 20.0}, {"bond_timeout": 8.0},
            {"node_names": ["controller_server","planner_server","behavior_server","bt_navigator"]},
        ],
    )

    base_to_d456_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="base_to_d456_tf",
    # x=0.457 (front), y=-0.289 (right of center), z=0.210 (height)
    # rpy=0 0 0 (forward-facing, level)
    arguments=["0.457", "-0.289", "0.210", "0", "0", "0",
               "base_link", "d456_link"],
    )

    base_to_d455_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_d455_tf",
        # x=-0.556 (behind rear), y=0.032 (slightly left), z=0.464 (height)
        # rpy: roll=0, pitch=0.5236 (30° down), yaw=3.14159 (rear-facing)
        arguments=["-0.556", "0.032", "0.464", "0", "0.0", "3.14159",
                "base_link", "d455_link"],
    )

    base_to_s3_lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_s3_lidar_tf",
        # x=0.0 (centered), y=0.102 (left of center), z=0.470 (height)
        arguments=["0.0", "0.102", "0.470", "0", "0", "0",
                "base_link", "s3_lidar_link"],
    )

    # ── NAVIGATION SERVERS ────────────────────────────────────────────────────
    excavation_server_node = Node(
        package="navigation_pkg", executable="excavation_server",
        name="excavation_server", output="screen",
    )
    localization_server_node = Node(
        package="navigation_pkg", executable="localization_server",
        name="localization_server", output="screen",
    )
    navigation_client_node = Node(
        package="navigation_pkg", executable="navigation_client",
        name="navigation_client", output="screen",
    )

    # ── HARDWARE ──────────────────────────────────────────────────────────────
    hardware_controller_module = Node(
        name="controller_node", package="controller_pkg", executable="controller_node",
    )
    depositing_module  = Node(name="depositing_node",  package="controller_pkg", executable="depositing_node")
    excavation_module  = Node(name="excavation_node",  package="controller_pkg", executable="excavation_node")
    health_module      = Node(name="health_node",      package="controller_pkg", executable="health_node")

    # ── WEB GUI ───────────────────────────────────────────────────────────────
    web_user_interface = Node(name="webgui_node", package="webgui_pkg", executable="webgui_server")
    rosbridge_node = Node(
        package="rosbridge_server", executable="rosbridge_websocket",
        name="rosbridge_websocket", parameters=[{"port": 9090}],
    )
    d456_compress_node = Node(
        package="image_transport", executable="republish", name="d456_republish",
        arguments=["raw", "compressed"],
        remappings=[("in", "/d456/color/image_raw"), ("out/compressed", "/d456/color/image_raw/compressed")],
        parameters=[{
            "use_sim_time": False,
            "compressed.jpeg_quality": 10, 
            "publish_frequency": 5.0,  # limit bandwidth usage

        }],
    )
    d455_compress_node = Node(
        package="image_transport", executable="republish", name="d455_republish",
        arguments=["raw", "compressed"],
        remappings=[("in", "/d455/color/image_raw"), ("out/compressed", "/d455/color/image_raw/compressed")],
        parameters=[{
            "use_sim_time": False,
            "compressed.jpeg_quality": 10,
            "publish_frequency": 5.0,  # limit bandwidth usage
        }],
    )

    # ══════════════════════════════════════════════════════════════════════════
    ld = LaunchDescription()
    ld.add_action(declare_robot_mode)

    # Always-on
    
    #ld.add_action(robot_state_publisher_node)
    ld.add_action(base_to_d456_tf)
    ld.add_action(base_to_d455_tf)
    ld.add_action(base_to_s3_lidar_tf)
    #ld.add_action(joint_state_publisher_node)
    ld.add_action(s3_lidar_node)
    ld.add_action(s3_filter_node)
    ld.add_action(d455_launch)
    ld.add_action(d456_launch)
    #ld.add_action()
    ld.add_action(imu_rotator_node)
    ld.add_action(d455_imu_filter)
    ld.add_action(d456_imu_filter)
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)
    ld.add_action(apriltag_d455_node)
    ld.add_action(apriltag_d456_node)
    ld.add_action(hardware_controller_module)
    ld.add_action(depositing_module)
    ld.add_action(excavation_module)
    ld.add_action(health_module)
    ld.add_action(web_user_interface)
    ld.add_action(rosbridge_node)
    ld.add_action(d456_compress_node)
    ld.add_action(d455_compress_node)

    # ── MANUAL MODE ───────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
        actions=[
            TimerAction(period=2.0, actions=[rf2o_odometry_node]),   # lidar must be up first
            TimerAction(period=4.0, actions=[ukf_node]),              # needs rf2o publishing
            TimerAction(period=8.0, actions=[slam_node]),             # needs /odometry/filtered
            TimerAction(period=20.0, actions=[
                controller_server_node, planner_server_node,
                behavior_server_node, bt_navigator_node, lifecycle_manager_node,
            ]),
        ],
    ))

    # ── AUTO MODE ─────────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
        actions=[
            TimerAction(period=2.0, actions=[rf2o_odometry_node]),
            TimerAction(period=4.0, actions=[ukf_node]),
            TimerAction(period=8.0, actions=[slam_node]),
            TimerAction(period=3.0, actions=[
                excavation_server_node, localization_server_node, navigation_client_node,
            ]),
            TimerAction(period=20.0, actions=[
                controller_server_node, planner_server_node,
                behavior_server_node, bt_navigator_node, lifecycle_manager_node,
            ]),
        ],
    ))

    return ld