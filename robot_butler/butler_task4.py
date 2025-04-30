#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

class OrderNavigator(Node):
    def __init__(self):
        super().__init__('order_navigator')
        
        # Subscriptions
        self.order_sub = self.create_subscription(String, '/order_request', self.order_callback, 10)
        self.cancel_sub = self.create_subscription(String, '/cancel_order', self.cancel_callback, 10)

        # Navigation
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # States
        self.state = 'idle'
        self.current_table = None
        self.hold_start_time = None

        # Locations
        self.locations = {
            'home': self.create_pose_stamped(0.0, -0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.78, 5.06, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
        }

        self.timer = self.create_timer(1.0, self.update)

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = float(yaw)  # Not accurate, just placeholder
        return pose

    def order_callback(self, msg):
        if self.state == 'idle':
            table = msg.data.strip()
            if table not in self.locations or not table.startswith('table'):
                self.get_logger().warn(f"Unknown table: {table}")
                return
            self.get_logger().info(f"Order received for {table}")
            self.current_table = table
            self.state = 'to_kitchen'
            self.navigator.goToPose(self.locations['kitchen'])

    def cancel_callback(self, msg):
        canceled_table = msg.data.strip()
        if canceled_table != self.current_table:
            self.get_logger().info(f"Ignored cancel for {canceled_table}. Current order is for {self.current_table}.")
            return

        if self.state == 'to_kitchen':
            self.get_logger().info(f"Order to {canceled_table} canceled while heading to kitchen. Returning home.")
            self.state = 'returning_home'
            self.navigator.cancelTask()
            self.navigator.goToPose(self.locations['home'])

        elif self.state == 'to_table':
            self.get_logger().info(f"Order to {canceled_table} canceled while heading to table. Returning to kitchen.")
            self.state = 'returning_kitchen'
            self.navigator.cancelTask()
            self.navigator.goToPose(self.locations['kitchen'])

    def update(self):
        if self.state in ['holding_kitchen', 'holding_table','holding_kitchen_cancel']:
            now = self.get_clock().now()
            if (now.nanoseconds - self.hold_start_time.nanoseconds) / 1e9 >= 5.0:
                if self.state == 'holding_kitchen':
                    self.get_logger().info("Hold at kitchen complete. Proceeding to table.")
                    self.state = 'to_table'
                    self.navigator.goToPose(self.locations[self.current_table])
                elif self.state == 'holding_table':
                    self.get_logger().info("Hold at table complete. Returning home.")
                    self.state = 'done'
                    self.navigator.goToPose(self.locations['home'])
                if self.state == 'holding_kitchen_cancel':
                    self.get_logger().info("Hold at kitchen complete. Returning home.")
                    self.state ='returning_home'
                    self.navigator.goToPose(self.locations['home'])

            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        if self.state == 'to_kitchen':
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Arrived at kitchen. Holding for 5 seconds.")
                self.state = 'holding_kitchen'
                self.hold_start_time = self.get_clock().now()
            else:
                self.get_logger().warn("Failed to reach kitchen. Returning home.")
                self.state = 'idle'
                self.navigator.goToPose(self.locations['home'])

        elif self.state == 'to_table':
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Arrived at table. Holding for 5 seconds.")
                self.state = 'holding_table'
                self.hold_start_time = self.get_clock().now()
            else:
                self.get_logger().warn("Failed to reach table. Returning home.")
                self.state = 'idle'
                self.navigator.goToPose(self.locations['home'])

        elif self.state == 'returning_kitchen':
            self.get_logger().info("Back at kitchen. Now returning home.")
            self.state = 'holding_kitchen_cancel'
            self.hold_start_time = self.get_clock().now()
            #self.navigator.goToPose(self.locations['home'])

        elif self.state == 'returning_home' or self.state == 'done':
            self.get_logger().info("Returned to home. Task complete.")
            self.state = 'idle'
            self.current_table = None

def main(args=None):
    rclpy.init(args=args)
    navigator_node = OrderNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(navigator_node)
    executor.spin()
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
