#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import time

class OrderNavigator(Node):
    def __init__(self):
        super().__init__('order_navigator')
        
        # Subscriptions
        self.create_subscription(String, '/table_orders', self.order_callback, 10)
        self.create_subscription(String, '/cancel_order', self.cancel_callback, 10)

        # Navigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # Locations
        self.locations = {
            'home': self.create_pose_stamped(0.0, -0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.78, 5.06, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
        }

        # State
        self.state = 'idle'
        self.pending_tables = []
        self.canceled_tables = set()
        self.current_index = 0

        self.timer = self.create_timer(1.0, self.update)

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = float(yaw)  # simplified orientation
        return pose

    def order_callback(self, msg):
        new_orders = [t.strip() for t in msg.data.split(',')]
        for order in new_orders:
            if order in self.locations and order not in self.pending_tables:
                self.pending_tables.append(order)
        if self.state == 'idle' and self.pending_tables:
            self.get_logger().info(f"Received orders: {self.pending_tables}")
            self.state = 'to_kitchen'
            self.navigator.goToPose(self.locations['kitchen'])

    def cancel_callback(self, msg):
        cancel_table = msg.data.strip()
        if cancel_table in self.locations:
            self.canceled_tables.add(cancel_table)
            self.get_logger().info(f"Order for {cancel_table} canceled.")
        else:
            self.get_logger().warn(f"Unknown table in cancel: {cancel_table}")

    def hold_in_place(self, seconds=5):
        self.get_logger().info(f"Holding for {seconds} seconds...")
        time.sleep(seconds)

    def update(self):
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        if self.state == 'to_kitchen':
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Arrived at kitchen.")
                self.hold_in_place()
                self.state = 'delivering'
                self.current_index = 0
                self.send_next_delivery()
            else:
                self.get_logger().warn("Failed to reach kitchen.")
                self.reset()

        elif self.state == 'delivering':
            if self.current_index == 0 or result == TaskResult.SUCCEEDED:
                last_table = self.pending_tables[self.current_index - 1] if self.current_index > 0 else 'none'
                self.get_logger().info(f"Delivered to {last_table}.")
                self.hold_in_place()
                self.send_next_delivery()
            else:
                self.get_logger().warn("Failed to deliver to a table. Skipping.")
                self.send_next_delivery()

        elif self.state == 'returning_kitchen':
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Returned to kitchen. Now heading home.")
                self.hold_in_place()
                self.state = 'returning_home'
                self.navigator.goToPose(self.locations['home'])

        elif self.state == 'returning_home':
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Returned home. Delivery cycle complete.")
            else:
                self.get_logger().warn("Failed to return home.")
            self.reset()

    def send_next_delivery(self):
        while self.current_index < len(self.pending_tables):
            table = self.pending_tables[self.current_index]
            self.current_index += 1
            if table not in self.canceled_tables:
                self.get_logger().info(f"Heading to {table}")
                self.navigator.goToPose(self.locations[table])
                return
            else:
                self.get_logger().info(f"{table} was canceled. Skipping.")
                self.navigator.cancelTask()
                self.state='delivering'
        self.get_logger().info("All deliveries done. Returning to kitchen.")
        self.state = 'returning_kitchen'
        self.navigator.goToPose(self.locations['kitchen'])

    def reset(self):
        self.state = 'idle'
        self.pending_tables = []
        self.canceled_tables.clear()
        self.current_index = 0

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
