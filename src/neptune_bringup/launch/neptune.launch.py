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

    # Add Webgui
    web_user_interface = Node(
        name        ="webgui_node",
        package     = "webgui_pkg",
        executable  = "webgui_server"
    )

    # Add Realsense Camera
    rs_camera_module = Node(
        name        ="camera_node",
        package     ="vision_pkg",
        executable  ="rs_camera_node"
    )

    # Add Localization Node
    localization_module = Node(
        package     ="localization_pkg",
        executable  ="localization_node"
    )

    # Add Hardware Controller
    hardware_controller_module = Node(
        name        ="controller_node",
        package     ="controller_pkg",
        executable  ="controller_node"
    )

    # Add Depositing Sequence
    depositing_module = Node(
        name        ="depositing_node",
        package     ="controller_pkg",
        executable  ="depositing_node",
    )

    # Add Excavation Sequence
    excavation_module = Node(
        name        ="excavation_node",
        package     ="controller_pkg",
        executable  ="excavation_node",
    )

    # Add Helath Node
    health_module = Node(
        name        ="health_node",
        package     ="controller_pkg",
        executable  ="health_node",
    )

    # Bucket sensor
    serial_reader_module = Node(
        name = "serial_reader_node",
        package = "controller_pkg",
        executable = "serial_reader_node"
    )

    # Add Logger Node
    logger_module = Node(
        name        ="logger_node",
        package     ="logger_pkg",
        executable  ="loggerNode"
    )

    # Add Actions to Launch Description
    ld.add_action(ros_bridge_server)
    # ld.add_action(madgwick_filter)
    ld.add_action(web_user_interface)
    ld.add_action(rs_camera_module)
    ld.add_action(hardware_controller_module)
    ld.add_action(depositing_module)
    ld.add_action(excavation_module)
    ld.add_action(localization_module)
    ld.add_action(health_module)
    ld.add_action(logger_module)
    ld.add_action(serial_reader_module)
    return ld
