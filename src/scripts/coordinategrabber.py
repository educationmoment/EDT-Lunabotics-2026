#!/usr/bin/env python3
"""
Coordinate Grabber for Real Robot
Run this while your robot is running to get coordinates for navigation goals.

Usage:
  1. Run your robot launch file in manual mode first
  2. Run this script: python3 coordinate_grabber.py
  3. Drive your robot to each goal position manually
  4. Press ENTER to record each position
  5. Copy the output to your navigation_client.cpp
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import TransformException
import sys


class CoordinateGrabber(Node):
    def __init__(self):
        super().__init__('coordinate_grabber')
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.positions = []
        
        # TF buffer for getting robot position in map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to odometry as backup
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribe to clicked points from RViz
        self.clicked_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.clicked_callback, 10)
        
        # Timer to update position from TF
        self.timer = self.create_timer(0.1, self.update_position)
        
        print("\n" + "="*60)
        print("COORDINATE GRABBER FOR REAL ROBOT")
        print("="*60)
        print("\nOptions:")
        print("  1. Drive robot to position, press ENTER to record")
        print("  2. Use RViz 'Publish Point' tool to click on map")
        print("  3. Type 'done' to finish and get C++ code")
        print("  4. Type 'clear' to clear all positions")
        print("  5. Type 'list' to show recorded positions")
        print("="*60 + "\n")

    def odom_callback(self, msg):
        # Backup if TF not available
        pass

    def clicked_callback(self, msg):
        """Handle clicked points from RViz"""
        x = msg.point.x
        y = msg.point.y
        self.positions.append((x, y))
        print(f"\n[RViz Click] Recorded position {len(self.positions)}: x={x:.3f}, y={y:.3f}")

    def update_position(self):
        """Get current robot position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            
            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y
            
            # Get yaw from quaternion
            q = transform.transform.rotation
            import math
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
        except TransformException:
            pass  # TF not available yet

    def record_current_position(self):
        """Record the current robot position"""
        self.positions.append((self.current_x, self.current_y))
        print(f"\nRecorded position {len(self.positions)}: x={self.current_x:.3f}, y={self.current_y:.3f}, yaw={self.current_yaw:.3f}")

    def print_cpp_code(self):
        """Print C++ code for navigation goals"""
        if not self.positions:
            print("\nNo positions recorded!")
            return
            
        print("\n" + "="*60)
        print("COPY THIS TO YOUR navigation_client.cpp:")
        print("="*60 + "\n")
        
        # Format for C++ array
        goals_str = ", ".join([f"{{{{{p[0]:.2f}, {p[1]:.2f}}}}}" for p in self.positions])
        print(f"const std::array<GoalXY, {len(self.positions)}> goals_{{{{{goals_str}}}}};")
        
        print("\n" + "="*60)
        print("INDIVIDUAL POSITIONS:")
        print("="*60)
        for i, (x, y) in enumerate(self.positions):
            print(f"  Goal {i}: x={x:.3f}, y={y:.3f}")
        print("="*60 + "\n")

    def clear_positions(self):
        """Clear all recorded positions"""
        self.positions = []
        print("\nCleared all positions.")

    def list_positions(self):
        """List all recorded positions"""
        if not self.positions:
            print("\nNo positions recorded yet.")
            return
        print(f"\nRecorded {len(self.positions)} positions:")
        for i, (x, y) in enumerate(self.positions):
            print(f"  {i}: x={x:.3f}, y={y:.3f}")


def main():
    rclpy.init()
    node = CoordinateGrabber()
    
    # Spin in background thread
    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        while True:
            # Show current position
            print(f"\rCurrent: x={node.current_x:.3f}, y={node.current_y:.3f} | "
                  f"Recorded: {len(node.positions)} | Press ENTER to record, 'done' to finish", 
                  end='', flush=True)
            
            user_input = input()
            
            if user_input.lower() == 'done':
                node.print_cpp_code()
                break
            elif user_input.lower() == 'clear':
                node.clear_positions()
            elif user_input.lower() == 'list':
                node.list_positions()
            elif user_input == '':
                node.record_current_position()
            else:
                print("Unknown command. Use ENTER, 'done', 'clear', or 'list'")
                
    except KeyboardInterrupt:
        print("\n\nInterrupted. Printing recorded positions...")
        node.print_cpp_code()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()