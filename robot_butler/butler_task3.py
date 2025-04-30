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
        
        # Locations (update with your actual coordinates)
        self.locations = {
            'home': self.create_pose_stamped(0.0, -0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.75, 5.18, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
            # Add more tables as needed
        }
        
        # Configuration parameters
        self.confirmation_timeout = 30.0  # seconds to wait for confirmation
        self.current_table = None
        self.received_confirmation = False
        self.current_location = 'home'
        self.confirmation_timer = None
        self.food_loaded = False  # Track if food was received from kitchen
        
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
        self.food_loaded = False
        self.publish_status(f"ORDER_RECEIVED: Table {self.current_table}")
        if self.current_table is not None :
           self.go_to_kitchen()

    def go_to_kitchen(self):
        self.publish_status("MOVING_TO_KITCHEN")
        self.navigator.goToPose(self.locations['kitchen'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = 'kitchen'
        self.wait_for_confirmation('kitchen')

    def wait_for_confirmation(self, location):

        self.cancel_timer_safe()
        self.publish_status(f"WAITING_AT_{location.upper()}_FOR_CONFIRMATION")
        self.confirmation_pub.publish(String(data=location))
        
        # Reset confirmation flag and start timer
        self.received_confirmation = False


       # if self.received_confirmation == False :    
        self.confirmation_timer = self.create_timer(
               self.confirmation_timeout, 
               lambda: self.on_confirmation_timeout(location)
            )
    def cancel_timer_safe(self):
        if self.confirmation_timer:
            try:
                self.confirmation_timer.cancel()
                self.get_logger().debug("Cancelled existing timer")
            except:
                self.get_logger().error("Error cancelling timer", exc_info=True)
            finally:
                self.confirmation_timer = None
    

    def handle_confirmation(self, msg):
        self.cancel_timer_safe()
    
        self.received_confirmation = msg.data
        if msg.data:  # if confirmation is True
            self.publish_status("CONFIRMATION_RECEIVED")
            if self.current_location == 'kitchen':
                self.food_loaded = True
                self.go_to_table()
            else:  # at table
                self.return_home()
        else:
            self.publish_status("CONFIRMATION_DENIED")
            self.handle_confirmation_failure()

    def handle_confirmation_failure(self):
        if self.current_location == 'kitchen':
            self.return_home()
        if self.current_location == 'home':
            self.get_logger().warn("wait for a new order")
        else:  # at table
            # Special requirement: Return to kitchen first before home
            self.return_to_kitchen()

    # def confirmation_timeout_callback(self, location):
    #     if self.current_location == 'kitchen' and self.current_location == self.current_table :
    #        self.publish_status(f"TIMEOUT_AT_{location.upper()}_NO_CONFIRMATION")
    #     self.confirmation_timer = None
    #     self.handle_confirmation_failure()
    def on_confirmation_timeout(self, location):
        self.get_logger().warning(f"Timeout at {location} - no confirmation received")
        self.cancel_timer_safe()
        self.publish_status(f"TIMEOUT_AT_{location.upper()}_NO_CONFIRMATION")
        self.handle_confirmation_failure()

    def go_to_table(self):
        self.publish_status(f"MOVING_TO_TABLE_{self.current_table}")
        self.navigator.goToPose(self.locations[f'table_{self.current_table}'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = f'table_{self.current_table}'
        self.wait_for_confirmation('table')

    def return_to_kitchen(self):
        self.publish_status("RETURNING_TO_KITCHEN_FIRST")
        self.navigator.goToPose(self.locations['kitchen'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        self.current_location = 'kitchen'
        self.return_home()

    # def return_home(self):
    #     self.publish_status("RETURNING_HOME")
    #     self.navigator.goToPose(self.locations['home'])
        
    #     while not self.navigator.isTaskComplete():
    #         pass
            
    #     self.current_location = 'home'
    #     self.current_table = None
    #     self.food_loaded = False
       

    #     self.received_confirmation = False
    #     if self.confirmation_timer:
    #        self.confirmation_timer.cancel()
    #        self.confirmation_timer = None

    #     self.publish_status("READY_AT_HOME")   

    def return_home(self):
        self.publish_status("RETURNING_HOME")
        self.navigator.goToPose(self.locations['home'])
        
        while not self.navigator.isTaskComplete():
            pass
            
        # Reset all state variables
        self.current_location = 'home'
        self.current_table = None
        self.food_loaded = False
        self.received_confirmation = False
        if self.confirmation_timer:
            self.confirmation_timer.cancel()
            self.confirmation_timer = None
        
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