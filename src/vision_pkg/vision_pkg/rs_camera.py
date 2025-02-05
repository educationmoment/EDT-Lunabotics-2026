""" 
    RS_CAMERA:
    
    Description:


"""

# Import Dependencies
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import rclpy
import pyrealsense2 as rs
import cv2 as cv

# Working on April Tag
import apriltag 

#########
# Camera Node
class CameraNode( Node ):

    # Initiallize Node
    def __init__(self, poll_period_sec: float=0.250) -> None:
        
        # Initiallize Camera Node 
        super().__init__("camera_node")
        self.get_logger().info("[ Camera.INIT ] Initiallized")

        # Initiallize Camera
        self.init_camera()

        # Initiallize Publisher
        self.init_comp_publisher(topic_name="compressed_video")

        # Initiallize AprilTag Detector
        self.init_detector()

        # Initialize Timer
        self.timer_counter = 0
        self.init_timer(period=poll_period_sec)
        return

    # Destructor
    def __del__(self) -> None:
        self.get_logger().info("[ Camera.DEL ] Destroying Camera Node")
        if self.pipeline: self.pipeline.stop()

        return

    ### CameraNode: CALLBACK FUNCTIONS  ###

    # Timer Callback Function
    def callback_timer(self) -> None:
        # self.get_logger().info(f"[ Camera.CALLBACK_TIMER ] Tick ({self.timer_counter})")
        self.timer_counter += 1

        # Collect Frames + Error Handling
        try:
            frames = self.pipeline.wait_for_frames()
        except Exception as e:
            self.get_logger().error(f"[ Camera.CALLBACK_TIMER ] Error: {e}")
            return
        
        # video_frame = frames.get_depth_frame()
        video_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        # video_frame = frames.get_infrared_frame()
        # video_frame = frames.get_fisheye_frame()

        # If Empty, Return
        if not video_frame: return

        # Collect Data
        image = video_frame.as_frame().get_data()
        np_image = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2RGB)



        # Collect AprilTag Location
        np_grayscale = cv.cvtColor( np.asanyarray(image) , cv.COLOR_BGR2GRAY )
        result = self.detector.detect(np_grayscale)
        if len(result) == 0:
            self.get_logger().warn("[ CameraNode.timer ] No tag detected")
        else:
            self.get_logger().info(f"[ CameraNode.timer ]Tag ID: {result[0].tag_id}")

            center = tuple(map(int, result[0].center))
            corner = tuple( map(list,result[0].corners) )

            cv.putText(np_image, "",center, cv.FONT_HERSHEY_SIMPLEX, 1, (50, 255, 40), 4)
            
            # cv.circle(np_image, center, 5, (255, 255, 0), 4)
            # cv.circle(np_image, tuple(map(int, result[0].center) ), 5, (255, 255, 0), 4)

            # Draw Corners onto image 
            # for i in range(4):
                # cv.circle(np_image, (int(corner[i][0]), int(corner[i][1]) ), 10, (255, 0, 0), 10)
            
            center_x, center_y = center

            # Get Distance to Center
            # self.get_logger().warn(f"[ DEBUG ] Length: {center}")
            # self.get_logger().warn(f"[ DEBUG ] Type: {type(map(int, result[0].center))}")


            # ISSUE: The grayscale image used to detect the april tag appears to have a different resolution
            #           than the depth frame.
            self.get_logger().info(f"[ DEBUG ] {center_x},{center_y}")
            if ( 0 <= center_x and center_x < depth_frame.get_width() ) and (0 <= center_y and center_y < depth_frame.get_height()):
                distance_center = depth_frame.get_distance( int(center_x), int(center_y) )
                self.get_logger().warn(f"Distance: {distance_center}")


            

        # Generate Message
        msg = CompressedImage()

        # Populate Message
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        # msg.data = cv.imencode( ".jpg", np_image )[1].flatten().tolist()

        # ChatGPT Suggested Fix
        msg.data = cv.imencode( ".jpg", np_image )[1].tobytes()

        # Send Message
        self.pub.publish(msg=msg)
        self.get_logger().info("[ Camera.CALLBACK_TIMER ] Frames Collected")
        return

    ### CameraNode: Initiallizers       ###

    # Initiallize AprilTag Detector
    def init_detector(self) -> None:
        self.get_logger().info("[ CameraNode.init_detector ] Initiallizing Detector")
        self.detector = apriltag.Detector()
        self.get_logger().info("[ CameraNode.init_detector ] Detector Initiallized")
        return

    # Initiallize Timer
    def init_timer(self, period: float) -> None:
        self.get_logger().info(f"[ Camera.INIT_TIMER ] Starting Timer, Period={period}")
        
        self.timer = self.create_timer(
            timer_period_sec=period,
            callback=self.callback_timer
        )
        self.get_logger().info("[ Camera.INIT_TIMER ] Timer Initiallized")
        return

    # Initiallize Camera Pipeline
    def init_camera(self) -> None:
        self.get_logger().info("[ Camera.INIT_CAMERA ] Starting Camera")

        # Initiallize Pipeline
        config = rs.config()
        config.enable_all_streams()
        # config.enable_stream(rs.stream.any, 640, 480, rs.format.z16, 30)

        self.pipeline = rs.pipeline()
        self.pipeline.start( config )
        self.get_logger().info("[ Camera.INIT_CAMERA ] Camera Initiallized")
        return

    # Initiallize Compressed Image Publisher
    def init_comp_publisher(self, topic_name: str) -> None:
        self.get_logger().info("[ Camera.INIT_COMP_PUBLISHER ] Starting Compressed Image Publisher")
        self.pub = self.create_publisher(
            msg_type = CompressedImage,
            topic = topic_name,
            qos_profile = 3
        )
        self.get_logger().info(f"[ Camera.INIT_COMP_PUBLISHER ] Publisher On \033[31m/{topic_name}\033[0m Initiallized")
        return


# Main Function
def main() -> int:
    rclpy.init()
    node = CameraNode(0.017)
    try:
        rclpy.spin(node=node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\n\033[100m[ Main.MAIN ] Keyboard Interrupt Pressed", end='\033[0m\n')

    return 0

if __name__ == "__main__":
    exit( main() )