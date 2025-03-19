""" 
    RS_CAMERA_Node:
    
        Description:
            Realsense Camera Node for handling a Single D455 Intel Realsense Camera.
            This node collects video and depth, as well as IMU acceleration and 
            gyroscope frames. It publishes to it's respective topics
            at a fixed frequency (i.e. 30FPS at 33 ms). A new frequency can be set
            by passing the desired period to the CameraNode.init_timer() method.
            


            Publishes four topics:
                Name                        Type                            Oneline-Description
            rs_node/compressed_video    sensor_msgs/msg/CompressedImage     Used to publish RGB uint8 video feed
            rs_node/depth_video         sensor_msgs/msg/Image               Used to publish depth camera footage
            rs_node/imu/accel_data      std_msgs/msg/Float32MultiArray      Used to publish x,y,z acceleration data
            rs_node/imu/gyro_data       std_msgs/msg/Float32MultiArray      Used to publish roll, pitch, yaw gyroscope data
"""

### Import Dependencies
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image



########################################
# TODO: Replace w/ interface type geometry_msgs/msg/AccelStamped
# from example_interfaces.msg import Float32MultiArray
from geometry_msgs.msg import AccelStamped

# TODO: Implement IMU interface to connect to Madgwick Filter Node 
from sensor_msgs.msg import Imu
########################################

# TODO: Implement /scan topic
from sensor_msgs.msg import LaserScan
########################################





# TODO: Implement Depth Camera Info Topic
from sensor_msgs.msg import CameraInfo
########################################

import numpy as np
import rclpy
import pyrealsense2 as rs
import cv2 as cv
import apriltag 

###################
# Camera Node
class CameraNode( Node ):
    ##########
    # Initiallize Node
    def __init__(self, poll_period_sec: float) -> None:
        
        ## Initiallize Camera Node 
        super().__init__("camera_node")
        self.get_logger().info("[ Camera.INIT ] Initiallized")

        ## Initiallize Camera
        self.init_camera()

        ## Initiallize Publisher
        self.init_node_publisher(
            camera_info_name    = "rs_node/camera/camera_info",         # Publishes Camera Info: Used by depthimage_to_laserscan node
            rgb_topic_name      = "rs_node/camera/compressed_video",    # Publishes Compressed RGB Image: Used by WebGUI
            depth_topic_name    = "rs_node/camera/depth_video",         # Publishes Depth Image: Used by depthimage_to_laserscan node
            imu_topic_name      = "imu/data_raw",                       # Publishes IMU Data: Used by Madgwick Filter Node
            scan_topic_name     = "scan"                               # Publishes Laser Scan Data: Used by Navigation Node
        )

        ## Initiallize AprilTag Detector
        self.init_detector()

        ## Initialize Timer
        self.timer_counter = 0
        self.start_time = self.get_clock().now()
        self.init_timer(period=poll_period_sec)
        return

    ##########
    # DONE: Destructor
    def __del__(self) -> None:
        self.get_logger().info("[ Camera.DEL ] Destroying Camera Node")

        ## Stop Pipeline
        if self.pipeline: self.pipeline.stop()
        return

    ### CameraNode: CALLBACK FUNCTIONS  ###
    ##########
    # In-Progress: Timer Callback Function
    def callback_timer(self) -> None:
        self.timer_counter += 1

        ## Collect Frames + Error Handling
        ######################################
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().error(f"[ Camera.CALLBACK_TIMER ] Error: {e}")
            return

        ## Process Frames
        ######################################
        ( video_frame, depth_frame, accl_data, gyro_data ) = self.process_frames( rs_frame=frames )
        
        ## Get NP Image out from Video Frame
        image = video_frame.as_frame().get_data()
        np_image = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2RGB)


        ## Send IMU data over Imu interface to Madgwick's Filter node
        ######################################
        self.send_imu_message( accel_data=accl_data, gyro_data=gyro_data )
        ######################################


        ## Collect AprilTag Location
        ######################################
        np_grayscale = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2GRAY )
        result = self.detector.detect(np_grayscale)
        if len(result) == 0:
            # self.get_logger().warn("[ CameraNode.timer ] No tag detected")
            pass
        else:
            self.get_logger().info(f"[ CameraNode.timer ]Tag ID: {result[0].tag_id}")

            # Collect Corner and Center Locations
            center = tuple(map(int, result[0].center))

            
            # Unpack Center Location
            center_x, center_y = center

            # Draw Corners and Center onto image 
            np_image = self.overlay_tag_corners(np_image, result[0])
            np_image = self.overlay_tag_center(np_image,  result[0])
            
            # Get Distance to Center
            # ISSUE: The grayscale image used to detect the april tag appears to have a different resolution
            #           than the depth frame.
            self.get_logger().info(f"[ DEBUG ] {center_x},{center_y}")
            if ( 0 <= center_x and center_x < depth_frame.get_width() ) and (0 <= center_y and center_y < depth_frame.get_height()):
                distance_center = depth_frame.get_distance( int(center_x), int(center_y) )
                cv.putText(np_image, f"{distance_center:0.3f}", (center_x - 20, center_y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 128, 40), 2)
        ######################################
        
        self.send_rgb_message(video_frame=video_frame, depth_frame=depth_frame)
        self.send_depth_message(depth_frame=depth_frame)
        ######################################
        return None

    ### CameraNode: Initiallizers       ###

    ##########
    # DONE: Initiallize AprilTag Detector
    def init_detector( self ) -> None:
        self.get_logger().info("[ CameraNode.init_detector ] Initiallizing Detector")
        
        ## Initiallize the AprilTag detector object
        self.detector = apriltag.Detector()
        self.get_logger().info("[ CameraNode.init_detector ] Detector Initiallized")
        return None

    ##########
    # DONE: Initiallize Timer
    def init_timer( self, period: float ) -> None:
        self.get_logger().info(f"[ Camera.INIT_TIMER ] Starting Timer, Period={period}")
        
        ## Initiallize the Timer Object
        #   - Period set by period parameter
        #       - Determines Frame Rate: 
        #               24FPS == 0.042
        #               30FPS == 0.033
        #               60FPS == 0.017
        #   - Callback set to self.callback_timer method
        self.timer = self.create_timer(
            timer_period_sec=period,
            callback=self.callback_timer
        )
        self.get_logger().info("[ Camera.INIT_TIMER ] Timer Initiallized")
        return None

    ##########
    # DONE: Initiallize Camera Pipeline
    def init_camera( self ) -> None:
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting Camera")
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting IMU")

        ## Initiallize Camera Pipeline
        ######################################
        self.pipeline = rs.pipeline()
        config = rs.config()
        ######################################


        ## Select, Configure, and Enable Streams in Pipeline configuration
        ######################################
        config.enable_stream(
            stream_type = rs.stream.color, 
            width       = 640, 
            height      = 480, 
            format      = rs.format.rgb8, 
            framerate  = 30
        )

        config.enable_stream(
            stream_type = rs.stream.depth, 
            width       = 640, 
            height      = 480, 
            format      = rs.format.z16,  
            framerate  = 30
        )

        config.enable_stream(
            stream_type = rs.stream.accel
            # format = rs.format.xyz32f,
            # framerate  = 30
        )

        config.enable_stream(
            stream_type = rs.stream.gyro
            # format = rs.format.xyz32f,
            # framerate  = 30
        )
        ######################################

        ## Send Configuration to Pipeline
        ######################################
        self.pipeline.start( config )
        self.get_logger().info("[ Camera.INIT_CAMERA ] Camera Initiallized")
        self.get_logger().info("[ Camera.INIT_CAMERA ] IMU Initiallized")
        ######################################
        return None

    ##########
    # DONE: Initiallize Compressed Image Publisher
    def init_node_publisher( self, camera_info_name: str, rgb_topic_name: str, depth_topic_name: str, imu_topic_name: str, scan_topic_name: str ) -> None:
        ##########
        # Initiallize the Camera Info Publisher
        #       Type == Compressed Image
        #       Topic == Specified in Function Call
        #       Quality of Service == 3 frames
        ######################################
        self.pub_cam_info = self.create_publisher(
            msg_type    = CameraInfo,
            topic       = camera_info_name,
            qos_profile = 5
        )

        ##########
        # Initiallize the RGB Camera Publisher
        #       Type == Compressed Image
        #       Topic == Specified in Function Call
        #       Quality of Service == 3 frames
        ######################################
        self.pub_rgb = self.create_publisher(
            msg_type = CompressedImage,
            topic = rgb_topic_name,
            qos_profile = 3
        )

        ##########
        # Initiallize the Depth Camera Publisher
        #       Type == Normal Image
        #       Topic == Specified in Function Call
        #       Quality of Service == 3 frames
        ######################################
        self.pub_depth = self.create_publisher(
            msg_type = Image,
            topic = depth_topic_name,
            qos_profile = 3
        )

        ##########
        # Initiallize IMU Publisher
        #       Type == sensor_msgs/msg/Imu
        #       Topic == imu_topic_name
        #       Quality of Service == 5 frames
        ######################################
        self.pub_imu = self.create_publisher(
            msg_type    = Imu,
            topic       = imu_topic_name,
            qos_profile = 5
        )

        ##########
        # Initiallize Scan Publisher
        #       Type == sensor_msgs/msg/LaserScan
        #       Topic == laser_scan_topic
        #       Quality of Service == 5 frames
        ######################################
        self.pub_scan = self.create_publisher(
            msg_type    = LaserScan,
            topic       = scan_topic_name,
            qos_profile = 5
        )
        return None

    ##########
    # DONE: Print Corners on AprilTag
    def overlay_tag_corners( self, np_image, tag) -> np.array:
        
        try:
            ## Determine of the tag passed is REALLY a tag:
            #   If NONE, then passed tag does not exist 
            ######################################
            if tag == None or np_image.size == 0:
                return np.zeros(1)
            ######################################


            ## Tag is real, what do we do now?
            ######################################
            else:
                # Collect Corners
                corners = tuple(map(list, tag.corners))

                # Print Corners to Image
                for i in range(4):
                    cv.circle(np_image, ( int(corners[i][0]), int(corners[i][1]) ), 10, (255, 0, 0), 10)

                # Return Image (Find out if it is possible to modify by reference)
                return np_image
            ######################################
        except BaseException as exc:
            self.get_logger().error(f"Error: {exc}")
        return np.zeros(1)

    ##########
    # DONE: Print Center on AprilTag
    def overlay_tag_center( self, np_image, tag):
        try:
            ## Determine of the tag passed is REALLY a tag:
            #   If NONE, then passed tag does not exist 
            ######################################
            if tag == None or np_image.size == 0:
                return np.zeros(1)
            ######################################


            ## Tag is real, what do we do now?
            ######################################
            else:
                # Collect Center
                center_X, center_Y = tuple(map(int, tag.center))

                # Print Center to Image
                cv.circle(np_image, (center_X, center_Y), 5, (255, 0, 255), 5)

                # Return Image (Find out if it is possible to modify by reference)
                return np_image    
            ######################################
        except BaseException as exc:
            self.get_logger().error(f"Error: {exc}")
        return np.zeros(1)
            
    ##########
    # DONE: Process Frames
    def process_frames( self, rs_frame: rs.composite_frame ) -> tuple:
        ## Collect Frames
        video_frame = rs_frame.first_or_default( rs.stream.color ).as_video_frame()
        depth_frame = rs_frame.first_or_default( rs.stream.depth ).as_depth_frame()
        accel_frame = rs_frame.first_or_default( rs.stream.accel ).as_motion_frame()
        gyro_frame = rs_frame.first_or_default( rs.stream.gyro ).as_motion_frame()

        ## Check for Empty Frames
        #   If any frame is empty, return None
        #   Else, return tuple of frames
        if not video_frame or not depth_frame or not accel_frame or not gyro_frame:
            return None
        else:
            video_frame.as_frame()
            accel_data = accel_frame.get_motion_data()
            gyro_data = gyro_frame.get_motion_data()
            return ( video_frame, depth_frame, accel_data, gyro_data )
        return None

    ##########
    # DONE: Send IMU Message
    def send_imu_message( self, accel_data: rs.vector, gyro_data: rs.vector ) -> None:
        ## Collect IMU Data
        (seconds, nano_seconds) = self.get_clock().now().seconds_nanoseconds()
        data = Imu()

        ## Populate IMU Header
        data.header.stamp.nanosec   = nano_seconds
        data.header.stamp.sec       = seconds
        data.header.frame_id        = f"{self.timer_counter}"

        ## Populate IMU Data: Angular Velocity
        data.angular_velocity.x     = gyro_data.x
        data.angular_velocity.y     = gyro_data.y
        data.angular_velocity.z     = gyro_data.z
        data.angular_velocity_covariance[0] = -1

        ## Populate IMU Data: Linear Acceleration
        data.linear_acceleration.x  = accel_data.x
        data.linear_acceleration.y  = accel_data.y
        data.linear_acceleration.z  = accel_data.z
        data.linear_acceleration_covariance[0] = -1

        ## Publish IMU Data
        self.pub_imu.publish(msg=data)
        return None

    ########
    # DONE: Send RGB Frame
    def send_rgb_message( self, video_frame: rs.frame, depth_frame: rs.frame ) -> None:
        ## Collect Image Data, Convert to NP Image
        image = video_frame.as_frame().get_data()
        np_image = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2RGB)

        ## Collect AprilTag Location
        grayscale_image =  cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2GRAY )

        ## Detect AprilTag
        #   - If no tag detected, return
        #   - Else, collect tag ID and center location
        #   - Draw corners and center onto image
        #   - Get distance to center
        result = self.detector.detect(grayscale_image)
        if len(result) == 0:
            self.get_logger().warn("[ CameraNode.timer ] No tag detected")
        else:
            self.get_logger().info(f"[ CameraNode.timer ]Tag ID: {result[0].tag_id}")

            # Collect Corner and Center Locations
            center = tuple(map(int, result[0].center))

            # Unpack Center Location
            center_x, center_y = center

            # Draw Corners and Center onto image 
            np_image = self.overlay_tag_corners(np_image, result[0])
            np_image = self.overlay_tag_center(np_image,  result[0])

            # Get Distance to Center
            # ISSUE: The grayscale image used to detect the april tag appears to have a different resolution
            #           than the depth frame.
            self.get_logger().info(f"[ DEBUG ] {center_x},{center_y}")
            if ( 0 <= center_x and center_x < depth_frame.get_width() ) and (0 <= center_y and center_y < depth_frame.get_height()):
                distance_center = depth_frame.get_distance( int(center_x), int(center_y) )
                cv.putText(np_image, f"{distance_center:0.3f}", (center_x - 20, center_y - 20), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 128, 40), 2)


        ## Create Message
        rgb_msg = CompressedImage()
        rgb_msg.header.stamp = self.get_clock().now().to_msg()
        rgb_msg.format = "jpeg"
        rgb_msg.data = cv.imencode( ".jpg", np_image )[1].tobytes()

        ## Publish Message
        self.pub_rgb.publish(msg=rgb_msg)
        return None

    ########
    # DONE: Send Depth Frame
    def send_depth_message( self, depth_frame: rs.frame ) -> None:

        ## Create Publisher Message
        depth_msg = Image()

        ## Populate Message
        depth_msg.header.stamp = self.get_clock().now().to_msg()
        depth_msg.header.frame_id = "camera_depth_frame"
        depth_msg.encoding = "16UC1"
        depth_msg.height = depth_frame.get_height()
        depth_msg.width = depth_frame.get_width()
        depth_msg.step = depth_msg.width * 2
        depth_msg.data = np.asanyarray(depth_frame.get_data()).tobytes()

        ## Publish Message
        self.pub_depth.publish(msg=depth_msg)
        return None

    ##########
    # TODO: Send Scan Message
    def send_scan_message(self) -> None:

        ## Create Publisher Message
        scan_msg = LaserScan()

        ## Populate Message
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "camera_depth_frame"
        scan_msg.angle_min = -3.14159
        scan_msg.angle_max = 3.14159
        scan_msg.angle_increment = 0.01
        scan_msg.time_increment = 0.01
        scan_msg.scan_time = 0.01
        scan_msg.range_min = 0.0
        scan_msg.range_max = 10.0

        # TODO: Implement Laser Scan Data
        scan_msg.ranges = np.zeros(628)
        scan_msg.intensities = np.zeros(628)

        ## Publish Message
        self.pub_scan.publish(msg=scan_msg)
        return None

#####################################
# Main Function
def main() -> int:
    rclpy.init()
    node = CameraNode(poll_period_sec=0.03334)


    ## Run Node   
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        print("\n\033[100m[ Main.MAIN ] Keyboard Interrupt Pressed", end='\033[0m\n')
    finally:
        node.__del__()
    return 0

if __name__ == "__main__":
    exit( main() )