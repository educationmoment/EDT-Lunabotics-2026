#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class PointLIORelay(Node):
    def __init__(self):
        super().__init__('point_lio_relay')
        self.sub = self.create_subscription(Odometry, '/point_lio/odom', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/point_lio/odom_cov', 10)

    def cb(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.covariance[0]  = 0.1
        msg.pose.covariance[7]  = 0.1
        msg.pose.covariance[35] = 0.1
        msg.twist.covariance[0]  = 0.1
        msg.twist.covariance[35] = 0.1
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(PointLIORelay())

if __name__ == '__main__':
    main()
