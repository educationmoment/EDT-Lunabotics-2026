#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler

class WaypointDemo(Node):
    def __init__(self):
        super().__init__('waypoint_demo')
        self.navigator = BasicNavigator()
        self.goals = []

        # Goal 1: move forward 1 meter
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.header.stamp = self.get_clock().now().to_msg()
        goal1.pose.position.x = 1.0
        goal1.pose.orientation.w = 1.0
        self.goals.append(goal1)

        # Goal 2: rotate 90° right in place
        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.header.stamp = self.get_clock().now().to_msg()
        goal2.pose.position.x = 1.0
        q = quaternion_from_euler(0, 0, -1.5708)  # -90 deg
        goal2.pose.orientation.x, goal2.pose.orientation.y = q[0], q[1]
        goal2.pose.orientation.z, goal2.pose.orientation.w = q[2], q[3]
        self.goals.append(goal2)

        self.run()

    def run(self):
        self.navigator.waitUntilNav2Active()
        self.navigator.followWaypoints(self.goals)
        while not self.navigator.isTaskComplete():
            self.get_logger().info('Navigating...')

        self.get_logger().info('Navigation complete.')

def main(args=None):
    rclpy.init(args=args)
    demo = WaypointDemo()
    rclpy.spin(demo)
    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

