import os
import xacro
import sys
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
    ExecuteProcess,
)


def generate_launch_description():
    config_dir       = get_package_share_directory("config_pkg")
    realsense_dir    = get_package_share_directory("realsense2_camera")

    nav2_params_file     = os.path.join(config_dir, "params", "nav2",               "nav2_uic_bot_params.yaml")
    rtabmap_params_file  = os.path.join(config_dir, "params", "rtabmap",             "rtabmap_params.yaml")
    ukf_params_file      = os.path.join(config_dir, "params", "robot_localization",  "ukf_params.yaml")
    #s3_params_file       = os.path.join(config_dir, "params", "laser_filters",       "s3_params.yaml")
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

    point_lio_relay_node = ExecuteProcess(
        cmd=[sys.executable,
            "/home/nuc/robot_WS/EDT-Lunabotics-2025/src/scripts/point_lio_relay.py"],
        output="screen",
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

    lidar_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_unilidar_tf",
        arguments=["-0.353", "0.0", "0.721",   # x y z in meters
                "0", "0", "3.14159",          # yaw pitch roll — roll=180° upside down
                "base_link", "s3_lidar_link"],
    )

    d435_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_d455_tf",
        arguments=["-0.634", "0.0", "0.703",   # x y z
                "0", "0", "3.14159",          # facing rear
                "base_link", "d455_link"],
    )
    # ── LIDAR ─────────────────────────────────────────────────────────────────
    #s3_lidar_node = Node(
    #    package="rplidar_ros", executable="rplidar_node", name="rplidar_node",
    #    output="screen",
    #    parameters=[{
    #        "channel_type": "serial", "serial_port": "/dev/ttyUSB0",
    #        "serial_baudrate": 1000000, "frame_id": "s3_lidar_link",
    #        "inverted": False, "scan_mode": "DenseBoost",
     #       "angle_compensate": True, "scan_frequency": 10.0,
     #   }],
     #   remappings=[("scan", "/scan_raw")],
    #)
    #s3_filter_node = Node(
    #    package="laser_filters", executable="scan_to_scan_filter_chain",
    #    parameters=[s3_params_file],
    #    remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    #)

    # ADD — replace the two rplidar nodes with this
    unitree_lidar_node = Node(
        package="unitree_lidar_ros2",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar",
        output="screen",
        parameters=[{
            "port": "/dev/ttyUSB0",   # udev rule name, or /dev/ttyUSB0
            "cloud_frame_id": "s3_lidar_link",
            "cloud_topic": "/unilidar/cloud",
            "imu_topic":   "/unilidar/imu",
            "rotate_yaw_bias": 3.14159,
            "range_min": 0.1,
            "range_max": 30.0,
            "cloud_flip": 1,
        }],
    )

    # ── CAMERAS ───────────────────────────────────────────────────────────────
    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, "launch", "rs_launch.py")),
        launch_arguments={
            "camera_name": "d455", "camera_namespace": "", "device_type": "d435i",
            "publish_tf": "true", "serial_no": "'337122075750'",
            "enable_color": "true",
            "enable_infra1": "true",
            "enable_depth": "true",
            "enable_gyro": "false", "enable_accel": "false", "unite_imu_method": "2",
            "depth_module.depth_profile": "848x480x30",   # was 1280x720x30
            "rgb_camera.color_profile":   "848x480x30",   # was 1280x720x30
            "pointcloud.enable": "true",
        }.items(),
    )
    d456_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_dir, "launch", "rs_launch.py")),
        launch_arguments={
            "camera_name": "d456", "camera_namespace": "", "device_type": "d455",
            "publish_tf": "true", "serial_no": "'308222300472'",
            "enable_color": "true",
            "enable_gyro": "false", "enable_accel": "false", "unite_imu_method": "2",
            "depth_module.depth_profile": "848x480x30",
            "rgb_camera.color_profile": "848x480x30",
            "pointcloud.enable": "true",
        }.items(),
    )

    webcam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="webcam",
        parameters=[{
            "video_device": "/dev/video6",   # change to your device
            "image_width":  640,
            "image_height": 480,
            "framerate":    30.0,
            "pixel_format": "mjpeg2rgb",     # most USB webcams use mjpeg
            "camera_frame_id": "webcam_link",
            "camera_name": "webcam",
        }],
        remappings=[
            ("image_raw",        "/webcam/image_raw"),
            ("camera_info",      "/webcam/camera_info"),
        ],
    )


    # ── IMU ───────────────────────────────────────────────────────────────────
    #imu_rotator_node = Node(package="util_pkg", executable="imu_rotator")
    #d455_imu_filter = Node(
    #    package="imu_complementary_filter", executable="complementary_filter_node",
    #    name="d455_imu_filter", output="screen",
    #    parameters=[{
    #        "publish_tf": False, "fixed_frame": "odom",
    #        "do_bias_estimation": True, "do_adaptive_gain": True,
    #        "use_mag": False, "gain_acc": 0.01, "gain_mag": 0.01,
    #    }],
    #    remappings=[("imu/data_raw", "/d455/imu/data_raw"), ("imu/data", "/d455/imu/data")],
    #)
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

    camera_init_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_init_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0",
                "odom", "camera_init"],
    )

    # ── RGBD SYNC ─────────────────────────────────────────────────────────────
    rgbd_sync1_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync1",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "approx_sync_max_interval": 0.05, "sync_queue_size": 1000, "qos_image": 2, "qos_camera_info": 2, "qos": 2}],
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

    webcam_compress_node = Node(
        package="image_transport",
        executable="republish",
        name="webcam_republish",
        arguments=["raw", "compressed"],
        remappings=[
            ("in",              "/webcam/image_raw"),
            ("out/compressed",  "/webcam/image_raw/compressed"),
        ],
        parameters=[{
            "compressed.jpeg_quality": 40,
        }],
    )

    # ── APRILTAG ──────────────────────────────────────────────────────────────
    #apriltag_d455_node = Node(
    #    package="apriltag_ros", executable="apriltag_node",
    #    name="apriltag_d455",          # ← give each a unique name
    #    output="screen",
    #    parameters=[apriltag_params_file],
    #    remappings=[("/image_rect", "/d455/color/image_raw"),
    #                ("/camera_info", "/d455/color/camera_info")],
    #)
    apriltag_d435_node = Node(
        package="apriltag_ros", executable="apriltag_node",
        name="apriltag_d435",
        output="screen",
        parameters=[apriltag_params_file],
        remappings=[
            #("/image_rect",  "/d455/color/image_raw"),
            #("/camera_info", "/d455/color/camera_info"),
            ("/image_rect",  "/d455/infra1/image_rect_raw"),   # IR stream
            ("/camera_info", "/d455/infra1/camera_info"),
        ],
    )
    apriltag_d456_node = Node(
        package="apriltag_ros", executable="apriltag_node",
        name="apriltag_d456",          # ← unique name
        output="screen",
        parameters=[apriltag_params_file],
        remappings=[("/image_rect", "/d456/color/image_raw"),
                    ("/camera_info", "/d456/color/camera_info")],
    )

    # ── ODOMETRY: rf2o ONLY → UKF ─────────────────────────────────────────────
    # Single source. Never run icp or rgbd_odom at the same time — causes UKF jumps.
    rf2o_odometry_node = Node(
        package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
        output="screen",
        respawn=False,
        parameters=[{
            "laser_scan_topic": "/scan", "odom_topic": "/rf2o_odom", #maybe change to scan_raw? idk
            "publish_tf": False,                 # rf2o owns odom TF (UKF bypassed)
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "laser_frame_id": "s3_lidar_link",  # real robot needs this
            "init_pose_from_topic": "",
            "freq": 10.0,
        }],
        arguments=["--ros-args", "--log-level", "error"],
    )

    point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("point_lio"),
                "config", "unilidar_l1.yaml"
            )
        ],
        remappings=[
            ("/cloud_registered", "/point_lio/cloud"),
            ("/aft_mapped_to_init", "/point_lio/odom"),
            ("/tf", "/point_lio/tf_internal")
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    rgbd_odom_node = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "publish_tf": False,        # UKF owns the TF
            "approx_sync": True,
            "approx_sync_max_interval": 0.05,  # was default 0.0 — set to 50ms to accept 33ms offset
            "sync_queue_size": 10,
            "Reg/Strategy": "0",        # visual only
            "Vis/MinInliers": "15",
            "OdomF2M/MaxSize": "1000",
        }],
        remappings=[
            ("rgb/image",       "/d456/color/image_raw"),
            ("depth/image",     "/d456/depth/image_rect_raw"),
            ("rgb/camera_info", "/d456/color/camera_info"),  # front camera
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )


    # ADD
    pc_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pc_to_scan",
        output="screen",
        parameters=[{
            #   "target_frame": "s3_lidar_link",
            "transform_tolerance": 0.01,
            "min_height": -0.5,
            "max_height": 0.1,
            "angle_min": -3.14159,
            "angle_max":  3.14159,
            "angle_increment": 0.00436,   # ~0.25 deg
            "scan_time": 0.1,
            "range_min": 0.3,
            "range_max": 30.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/unilidar/cloud"),
            ("scan",     "/scan"),
        ],
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
                "rgbd_cameras": 1,
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
                "subscribe_scan_cloud": True, "subscribe_scan": False, 
                "wait_imu_to_init": False,      # real robot: IMU already stable at launch
                "imu_topic": "/unilidar/imu",
                "qos_image": 2, "qos_camera_info": 2, "qos": 2,
                "tf_delay": 0.0,
                "tf_tolerance": 0.3,
            },
        ],
        remappings=[
            ("rgbd_image", "/d456/rgbd_image"),
            #("rgbd_image1", "/d455/rgbd_image"),
            ("scan_cloud",  "/point_lio/cloud"),      # Point-LIO registered cloud
            #("scan", "/scan"),
        ],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    apriltag_to_landmarks_node = Node(
        package="util_pkg",
        executable="apriltag_to_landmarks",
        name="apriltag_to_landmarks",
        output="screen",
        parameters=[{
           "tag_linear_variance":  0.01,   
            "tag_angular_variance": 0.05,
            "reference_frame": "base_link",
        }],
    )

    crater_scan_d435_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="crater_scan_d435",
        output="screen",
        remappings=[
            ("depth",             "/d455/depth/image_rect_raw"),
            ("depth_camera_info", "/d455/depth/camera_info"),
            ("scan",              "/crater_scan_rear"),
        ],
        parameters=[{
            "scan_height":    10,     # rows of depth image to sample
            "scan_time":      0.1,
            "range_min":      0.3,
            "range_max":      2.5,    # D455 at 0.37m height, 15deg down → ground at ~1.4m
            "output_frame":   "d455_depth_optical_frame",
        }],
    )

    crater_scan_d456_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="crater_scan_d456",
        output="screen",
        remappings=[
            ("depth",             "/d456/depth/image_rect_raw"),
            ("depth_camera_info", "/d456/depth/camera_info"),
            ("scan",              "/crater_scan_front"),
        ],
        parameters=[{
            "scan_height":    10,
            "scan_time":      0.1,
            "range_min":      0.2,
            "range_max":      1.5,    # D456 at 0.141m height, 15deg down → ground at ~0.53m
            "output_frame":   "d456_depth_optical_frame",
        }],
    )

    rgbd_sync3_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync3",
        output="screen",
        parameters=[{"use_sim_time": False, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[
            ("rgb/image",       "/d455/color/image_raw"),
            ("depth/image",     "/d455/depth/image_rect_raw"),
            ("rgb/camera_info", "/d455/color/camera_info"),
            ("rgbd_image",      "/d455/rgbd_image"),
        ],
        namespace="d455",
        arguments=["--ros-args", "--log-level", "error"],
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

    #base_to_d456_tf = Node(
    #package="tf2_ros",
    #executable="static_transform_publisher",
    #name="base_to_d456_tf",
    # x=0.457 (front), y=-0.289 (right of center), z=0.210 (height)
    # rpy=0 0 0 (forward-facing, level)
   # arguments=["0.457", "-0.289", "0.210", "0", "0", "0",
   #            "base_link", "d456_link"],
   # )

    #base_to_d455_tf = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="base_to_d455_tf",
    #    # x=-0.556 (behind rear), y=0.032 (slightly left), z=0.464 (height)
    #    # rpy: roll=0, pitch=0.5236 (30° down), yaw=3.14159 (rear-facing)
    #    arguments=["-0.556", "0.032", "0.464", "0", "0.0", "3.14159",
    #            "base_link", "d455_link"],
    #)

   # base_to_s3_lidar_tf = Node(
    #    package="tf2_ros",
    #    executable="static_transform_publisher",
    #    name="base_to_s3_lidar_tf",
    #    # x=0.0 (centered), y=0.102 (left of center), z=0.470 (height)
    #    arguments=["0.0", "0.102", "0.470", "0", "0", "0",
    #            "base_link", "s3_lidar_link"],
   # )

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
    
    ld.add_action(robot_state_publisher_node)
    #ld.add_action(base_to_d456_tf)
    #ld.add_action(base_to_d455_tf)
    #ld.add_action(base_to_s3_lidar_tf)
    ld.add_action(joint_state_publisher_node)
    #ld.add_action(s3_lidar_node)
    #ld.add_action(s3_filter_node)
    ld.add_action(camera_init_tf_node)
    ld.add_action(lidar_tf_node)
    ld.add_action(d435_tf_node)
    ld.add_action(unitree_lidar_node)
    ld.add_action(pc_to_scan_node)
    ld.add_action(d455_launch)
    ld.add_action(d456_launch)
    ld.add_action(webcam_node)
    ld.add_action(webcam_compress_node)

    ld.add_action(apriltag_to_landmarks_node)
    #ld.add_action()
    #ld.add_action(imu_rotator_node)
    #ld.add_action(d455_imu_filter)
    #ld.add_action(d456_imu_filter)
    ld.add_action(rgbd_sync1_node)
    #ld.add_action(rgbd_sync2_node)
    #ld.add_action(apriltag_d455_node)
    ld.add_action(apriltag_d435_node)
    ld.add_action(apriltag_d456_node)
    ld.add_action(crater_scan_d435_node)
    ld.add_action(crater_scan_d456_node)
    #ld.add_action(rgbd_sync3_node)
    #ld.add_action(d455_filter_node)
    #ld.add_action(d456_filter_node)
    ld.add_action(hardware_controller_module)
    ld.add_action(depositing_module)
    ld.add_action(excavation_module)
    ld.add_action(health_module)
    ld.add_action(web_user_interface)
    ld.add_action(rosbridge_node)
    ld.add_action(d456_compress_node)
    ld.add_action(d455_compress_node)
    #ld.add_action(TimerAction(period=2.0,  actions=[rf2o_odometry_node]))
    ld.add_action(TimerAction(period=3.0,  actions=[point_lio_node]))
    ld.add_action(TimerAction(period=4.0, actions=[point_lio_relay_node]))
    ld.add_action(TimerAction(period=5.0,  actions=[rgbd_odom_node]))
    ld.add_action(TimerAction(period=10.0, actions=[ukf_node]))    # was 4.0
    ld.add_action(TimerAction(period=15.0, actions=[slam_node]))   # was 8.0

    # ── MANUAL MODE ───────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
        actions=[
            #TimerAction(period=2.0, actions=[rf2o_odometry_node]),   # lidar must be up first
            #TimerAction(period=4.0, actions=[ukf_node]),              # needs rf2o publishing
            #TimerAction(period=8.0, actions=[slam_node]),             # needs /odometry/filtered
            TimerAction(period=30.0, actions=[                        # wait longer for RTAB-Map to build map
                controller_server_node, planner_server_node,
                behavior_server_node, bt_navigator_node, lifecycle_manager_node,
            ]),
        ],
    ))

    # ── AUTO MODE ─────────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
        actions=[
            #TimerAction(period=2.0, actions=[rf2o_odometry_node]),
            #TimerAction(period=4.0, actions=[ukf_node]),
            #TimerAction(period=8.0, actions=[slam_node]),
            TimerAction(period=3.0, actions=[
                excavation_server_node, localization_server_node,
            ]),
            TimerAction(period=40.0, actions=[
                navigation_client_node,   # ← after Nav2 is fully active
            ]),
            TimerAction(period=35.0, actions=[                        # wait longer for RTAB-Map to build map
                controller_server_node, planner_server_node,
                behavior_server_node, bt_navigator_node, lifecycle_manager_node,
            ]),
        ],
    ))

    return ld
