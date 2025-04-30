#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_simple_commander.robot_navigator import BasicNavigator

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')
        
        # Initialize the navigator
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        
        # Publishers and Subscribers
        self.status_pub = self.create_publisher(String, '/robot_status', 10)
        self.confirmation_pub = self.create_publisher(String, '/request_confirmation', 10)
        self.order_sub = self.create_subscription(String, '/new_order', self.process_order, 10)
        self.confirmation_sub = self.create_subscription(Bool, '/confirmation_response', self.handle_confirmation, 10)
        
        # Locations (update with your coordinates)
        self.locations = {
            'home': self.create_pose_stamped(0.0, -0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.75, 5.18, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
        }
        
        # Configuration parameters
        self.confirmation_timeout = 30.0  # seconds to wait for confirmation
        self.current_table = None
        self.received_confirmation = False
        self.current_location = 'home'
        self.confirmation_timer = None
        
        self.get_logger().info("Delivery Robot initialized and ready for orders!")

    def create_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = float(yaw)
        return pose

    def process_order(self, msg):
        if self.current_location != 'home':
            self.get_logger().warn("Robot is not at home! Cannot accept new order.")
            return
            
        self.current_table = msg.data
        self.publish_status(f"ORDER_RECEIVED: Table {self.current_table}")
        self.go_to_kitchen()

    def go_to_kitchen(self):
        self.publish_status("MOVING_TO_KITCHEN")
        self.navigator.goToPose(self.locations['kitchen'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = 'kitchen'
        self.wait_for_confirmation('kitchen')

    def wait_for_confirmation(self, location):
        self.publish_status(f"WAITING_AT_{location.upper()}_FOR_CONFIRMATION")
        self.confirmation_pub.publish(String(data=location))
        
        # Reset confirmation flag and start timer
        self.received_confirmation = False
        if self.confirmation_timer:
            self.confirmation_timer.cancel()
        if self.received_confirmation == False :  
            self.confirmation_timer = self.create_timer(
               self.confirmation_timeout, 
               lambda: self.confirmation_timeout_callback(location)
            )

    def handle_confirmation(self, msg):
        if self.confirmation_timer is not None:
            self.confirmation_timer.cancel()
            self.confirmation_timer = None
    
        self.received_confirmation = msg.data
        if msg.data:  # if confirmation is True
            self.publish_status("CONFIRMATION_RECEIVED")
            if self.current_location == 'kitchen':
                self.go_to_table()
            else:  # at table
                self.return_home()
        else:
            self.publish_status("CONFIRMATION_DENIED")
            self.return_home()

    def confirmation_timeout_callback(self, location):
        self.publish_status(f"TIMEOUT_AT_{location.upper()}_NO_CONFIRMATION")
        self.confirmation_timer = None
        self.return_home()

    def go_to_table(self):
        self.publish_status(f"MOVING_TO_TABLE_{self.current_table}")
        self.navigator.goToPose(self.locations[f'table_{self.current_table}'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = f'table_{self.current_table}'
        self.wait_for_confirmation('table')

    def return_home(self):
        self.publish_status("RETURNING_HOME")
        self.navigator.goToPose(self.locations['home'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = 'home'
        self.current_table = None
        self.publish_status("READY_AT_HOME")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    delivery_robot = DeliveryRobot()
    rclpy.spin(delivery_robot)
    delivery_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()