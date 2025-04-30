#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
import time  # Import time module for delays

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()
        
        # Wait for navigation to fully activate
        self.navigator.waitUntilNav2Active()
        
        # Create publisher for status updates
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        
        # Create subscriber for new orders
        self.order_sub = self.create_subscription(
            String,
            '/new_order',
            self.process_order,
            10)
        
        # Define important locations (update with your actual coordinates)
        self.locations = {
            'home': self.create_pose_stamped(0.0, -0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.75, 5.18, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
            # Add more tables as needed
        }
        
        # Set the delay time (in seconds)
        self.movement_delay = 10  # 10 seconds between movements
        
        self.get_logger().info("Delivery Robot ready for orders!")
    
    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = float(yaw)
        return pose
    
    def process_order(self, msg):
        table = msg.data
        self.get_logger().info(f"Received order for {table}")
        
        # 1. Go to kitchen
        self.navigator.goToPose(self.locations['kitchen'])
        self.get_logger().info("Moving to kitchen...")
        while not self.navigator.isTaskComplete():
            pass
        
        # Add delay after reaching kitchen
        self.get_logger().info(f"Reached kitchen. Waiting for {self.movement_delay} seconds...")
        time.sleep(self.movement_delay)
        
        # 2. Go to the table
        self.navigator.goToPose(self.locations[f'table_{table}'])
        self.get_logger().info(f"Moving to table {table}...")
        while not self.navigator.isTaskComplete():
            pass
        
        # Add delay after reaching table
        self.get_logger().info(f"Reached table {table}. Waiting for {self.movement_delay} seconds...")
        time.sleep(self.movement_delay)
        
        # 3. Return home
        self.navigator.goToPose(self.locations['home'])
        self.get_logger().info("Returning home...")
        while not self.navigator.isTaskComplete():
            pass
        
        self.get_logger().info(f"Delivery to {table} completed!")

def main(args=None):
    rclpy.init(args=args)
    delivery_robot = DeliveryRobot()
    rclpy.spin(delivery_robot)
    delivery_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()