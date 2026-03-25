import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
    ExecuteProcess,
)


def generate_launch_description():
    config_dir        = get_package_share_directory("config_pkg")
    nav2_params_file  = os.path.join(config_dir, "params", "nav2",               "nav2_uic_bot_params.yaml")
    rtabmap_params    = os.path.join(config_dir, "params", "rtabmap",             "rtabmap_params.yaml")
    s3_params         = os.path.join(config_dir, "params", "laser_filters",       "s3_params.yaml")
    s2l_params        = os.path.join(config_dir, "params", "laser_filters",       "s2l_params.yaml")
    ukf_params        = os.path.join(config_dir, "params", "robot_localization",  "ukf_sim_params.yaml")
    apriltag_params   = os.path.join(config_dir, "params", "apriltag",            "tag_params.yaml")
    bt_nav_to_pose    = os.path.join(config_dir, "behavior_trees", "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml")
    bt_nav_through    = os.path.join(config_dir, "behavior_trees", "nav_through_poses_w_replanning_and_recovery.xml")

    # ── DECLARATIONS ──────────────────────────────────────────────────────────
    declare_robot_mode  = DeclareLaunchArgument("robot_mode",  default_value="manual", choices=["manual","auto"])
    declare_teleop_mode = DeclareLaunchArgument("teleop_mode", default_value="keyboard", choices=["keyboard"])
    declare_robot_type  = DeclareLaunchArgument("robot_type",  default_value="bulldozer", choices=["bulldozer"])

    # ── ALWAYS-ON ─────────────────────────────────────────────────────────────
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
            "use_sim_time": True,
            "compressed.jpeg_quality": 20, 

        }],
    )
    d455_compress_node = Node(
        package="image_transport", executable="republish", name="d455_republish",
        arguments=["raw", "compressed"],
        remappings=[("in", "/d455/color/image_raw"), ("out/compressed", "/d455/color/image_raw/compressed")],
        parameters=[{
            "use_sim_time": True,
            "compressed.jpeg_quality": 20,
        }],
    )

    topic_remapper_node = Node(package="util_pkg", executable="topic_remapper")

    rgbd_sync1_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync1",
        parameters=[{"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[("rgb/image", "/d456/color/image_raw"), ("depth/image", "/d456/depth/image_rect_raw"),
                    ("rgb/camera_info", "/d456/color/camera_info"), ("rgbd_image", "rgbd_image")],
        namespace="d456", arguments=["--ros-args", "--log-level", "error"],
    )
    rgbd_sync2_node = Node(
        package="rtabmap_sync", executable="rgbd_sync", name="rgbd_sync2",
        parameters=[{"use_sim_time": True, "approx_sync": True, "sync_queue_size": 1000}],
        remappings=[("rgb/image", "/d455/color/image_raw"), ("depth/image", "/d455/depth/image_rect_raw"),
                    ("rgb/camera_info", "/d455/color/camera_info"), ("rgbd_image", "rgbd_image")],
        namespace="d455", arguments=["--ros-args", "--log-level", "error"],
    )

    apriltag_d455_node = Node(
        package="apriltag_ros", executable="apriltag_node",
        parameters=[apriltag_params],
        remappings=[("/image_rect", "/d455/color/image_raw"), ("/camera_info", "/d455/color/camera_info")],
    )
    apriltag_d456_node = Node(
        package="apriltag_ros", executable="apriltag_node",
        parameters=[apriltag_params],
        remappings=[("/image_rect", "/d456/color/image_raw"), ("/camera_info", "/d456/color/camera_info")],
    )

    # Static map→odom TF placeholder until rtabmap takes over
    map_to_odom_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="map_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )

    # ── ODOMETRY: rf2o ONLY → UKF ─────────────────────────────────────────────
    # Using rf2o as the single odometry source.
    # icp_odometry is commented out — only uncomment one or the other, never both.
    rf2o_odometry_node = Node(
        package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        parameters=[{
            "laser_scan_topic": "/scan",
            "odom_topic":       "/rf2o_odom",
            "publish_tf":       False,          # UKF owns the odom TF
            "base_frame_id":    "base_link",
            "odom_frame_id":    "odom",
            "init_pose_from_topic": "",
            "freq": 20.0,
        }],
        arguments=["--ros-args", "--log-level", "error"],
    )

    # icp_odometry_node = Node(
    #     package="rtabmap_odom", executable="icp_odometry",
    #     parameters=[{
    #         "use_sim_time": True,
    #         "frame_id": "base_link", "odom_frame_id": "odom",
    #         "publish_tf": False, "guess_from_tf": True, "approx_sync": True,
    #         "Reg/Strategy": "1", "Odom/Strategy": "1",
    #         "Odom/FilteringStrategy": "1",
    #         "Odom/KalmanProcessNoise": "0.001", "Odom/KalmanMeasurementNoise": "0.01",
    #         "Icp/PointToPlane": "true", "Icp/Iterations": "10", "Icp/VoxelSize": "0.1",
    #         "Icp/Epsilon": "0.001", "Icp/PointToPlaneK": "20", "Icp/PointToPlaneRadius": "0",
    #         "Icp/MaxTranslation": "2", "Icp/MaxCorrespondenceDistance": "1",
    #         "Icp/Strategy": "1", "Icp/OutlierRatio": "0.7", "Icp/CorrespondenceRatio": "0.01",
    #         "Odom/ScanKeyFrameThr": "0.4", "OdomF2M/ScanSubtractRadius": "0.1",
    #         "OdomF2M/ScanMaxSize": "15000", "OdomF2M/BundleAdjustment": "false",
    #     }],
    #     remappings=[("scan", "/scan"), ("odom", "/icp_odom")],
    #     arguments=["--ros-args", "--log-level", "error"],
    # )

    ukf_node = Node(
        package="robot_localization", executable="ukf_node", name="ukf_filter_node",
        parameters=[{"use_sim_time": True}, ukf_params],
    )

    # ── LASER FILTERS ─────────────────────────────────────────────────────────
    s3_filter_node = Node(
        package="laser_filters", executable="scan_to_scan_filter_chain",
        parameters=[s3_params],
        remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
    )
    s2l_filter_node = Node(
        package="laser_filters", executable="scan_to_scan_filter_chain",
        parameters=[s2l_params],
        remappings=[("scan", "/scan2_raw"), ("scan_filtered", "/scan2")],
    )
    scan2_relay_node = Node(
        package="topic_tools", executable="relay", name="scan2_relay",
        parameters=[{"input_topic": "/scan2", "output_topic": "/scan"}],
    )

    # ── SLAM ──────────────────────────────────────────────────────────────────
    slam_node = Node(
        package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="log",
        parameters=[{
            "use_sim_time":       True,
            "rgbd_cameras":       2,
            "subscribe_depth":    False,
            "subscribe_rgbd":     True,
            "subscribe_rgb":      False,
            "subscribe_odom_info":False,
            "odom_sensor_sync":   True,
            "frame_id":           "base_link",
            "map_frame_id":       "map",
            "odom_frame_id":      "odom",
            "publish_tf":         True,
            "publish_tf_odom":    False,    # UKF owns odom TF, rtabmap publishes map→odom
            "database_path":      "",
            "approx_sync":        True,
            "sync_queue_size":    1000,
            "subscribe_scan_cloud": False,
            "subscribe_scan":     True,
            "wait_imu_to_init":   True,
            "imu_topic":          "/teensy/imu/data",
        }, rtabmap_params],
        remappings=[
            ("rgbd_image0", "/d456/rgbd_image"),
            ("rgbd_image1", "/d455/rgbd_image"),
            ("scan",        "/scan"),
            ("odom",        "/odometry/filtered"),  # rtabmap reads UKF output
        ],
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
            "default_nav_to_pose_bt_xml":       bt_nav_to_pose,
            "default_nav_through_poses_bt_xml": bt_nav_through,
        }],
    )
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", output="screen",
        parameters=[
            {"use_sim_time": True}, {"autostart": True}, {"node_timeout": 60.0},
            {"node_names": ["controller_server","planner_server","behavior_server","bt_navigator"]},
        ],
    )

    localization_server_node = Node(
        package="navigation_pkg", executable="localization_server",
        name="localization_server", output="screen",
    )
    navigation_client_node = Node(
        package="navigation_pkg", executable="navigation_client",
        name="navigation_client", output="screen",
    )

    # ── TELEOP ────────────────────────────────────────────────────────────────
    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node",
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )
    teleop_twist_joy_node = Node(
        package="teleop_twist_joy", executable="teleop_node", name="teleop_twist_joy_node",
        parameters=[{"require_enable_button": False, "axis_linear.x": 1,
                     "axis_angular.yaw": 0, "enable_turbo_button": 5,
                     "scale_linear_turbo.x": 1.5, "scale_angular_turbo.yaw": 1.5}],
        condition=LaunchConfigurationEquals("teleop_mode", "xbox"),
    )
    keyboard_teleop_node = ExecuteProcess(
        cmd=["gnome-terminal", "--", "bash", "-c", "ros2 run teleop_pkg keyboard_teleop.py; exit"],
        output="screen",
        condition=LaunchConfigurationEquals("teleop_mode", "keyboard"),
    )

    # ══════════════════════════════════════════════════════════════════════════
    ld = LaunchDescription()

    ld.add_action(declare_robot_mode)
    ld.add_action(declare_teleop_mode)
    ld.add_action(declare_robot_type)

    # Always-on nodes
    ld.add_action(web_user_interface)
    ld.add_action(rosbridge_node)
    ld.add_action(d456_compress_node)
    ld.add_action(d455_compress_node)
    ld.add_action(topic_remapper_node)
    ld.add_action(rgbd_sync1_node)
    ld.add_action(rgbd_sync2_node)
    ld.add_action(apriltag_d455_node)
    ld.add_action(apriltag_d456_node)
    ld.add_action(map_to_odom_tf)

    # ── MANUAL MODE ───────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "manual"),
        actions=[
            # T+2s: laser filters + odometry (rf2o only)
            TimerAction(period=2.0, actions=[
                s3_filter_node,
                s2l_filter_node,
                scan2_relay_node,
                rf2o_odometry_node,     # single odom source
            ]),
            # T+4s: UKF (needs rf2o to be publishing first)
            TimerAction(period=4.0, actions=[
                ukf_node,
            ]),
            # T+8s: SLAM (needs /odometry/filtered from UKF)
            TimerAction(period=8.0, actions=[
                slam_node,
            ]),
            # T+25s: Nav2
            TimerAction(period=25.0, actions=[
                controller_server_node,
                planner_server_node,
                behavior_server_node,
                bt_navigator_node,
                lifecycle_manager_node,
            ]),
            joy_node,
            teleop_twist_joy_node,
            keyboard_teleop_node,
        ],
    ))

    # ── AUTO MODE ─────────────────────────────────────────────────────────────
    ld.add_action(GroupAction(
        condition=LaunchConfigurationEquals("robot_mode", "auto"),
        actions=[
            TimerAction(period=2.0, actions=[
                s3_filter_node,
                s2l_filter_node,
                scan2_relay_node,
                rf2o_odometry_node,
            ]),
            TimerAction(period=4.0, actions=[
                ukf_node,
            ]),
            TimerAction(period=8.0, actions=[
                slam_node,
            ]),
            TimerAction(period=3.0, actions=[
                localization_server_node,
                navigation_client_node,
            ]),
            TimerAction(period=15.0, actions=[
                controller_server_node,
                planner_server_node,
                behavior_server_node,
                bt_navigator_node,
                lifecycle_manager_node,
            ]),
        ],
    ))

    return ld