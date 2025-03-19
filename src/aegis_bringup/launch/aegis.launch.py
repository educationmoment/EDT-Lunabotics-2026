from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ### Create Actions ###

    # Add Rosbridge Websocket
    ros_bridge_server = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket"
    )

    # Madgwick's Filter Node
    madgwick_filter = Node(
        name            = "madgwick_node",
        package         = "imu_filter_madgwick",
        executable      = "imu_filter_madgwick_node",
        parameters = [{
            "use_mag": False,
            "remove_gravity_vector": True,
            "gain": 0.0025,
            "frequency": 800
        }]
    )

    # Add Webgui
    web_user_interface = Node(
        package     = "webgui_pkg",
        executable  = "webgui_server"
    )
    
    # Add Realsense Camera
    rs_camera_module = Node(
        package="vision_pkg",
        executable="rs_camera_node"
    )

    # Add Localization Node
    localization_module = Node(
        package="localization_pkg",
        executable="localization_node"
    )

    # Add Hardware Controller
    hardware_controller_module = Node(
        package="controller_pkg",
        executable="controller_node"
    )


    
    # Example Talker/Listener Launch Description
    # talker_node = Node(
    #     package="demo_nodes_cpp",
    #     executable="talker",
    # )
    # 
    # listener_node = Node(
    #     package="demo_nodes_py",
    #     executable="listener"
    # )
    # ld.add_action(talker_node)
    # ld.add_action(listener_node)


    # Add Actions to Launch Description
    ld.add_action(ros_bridge_server)
    ld.add_action(madgwick_filter)
    ld.add_action(web_user_interface)
    ld.add_action(rs_camera_module)
    ld.add_action(hardware_controller_module)
    ld.add_action(localization_module)
    return ld


