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
            'home': self.create_pose_stamped(0.0, 0.0, 0.0),
            'kitchen': self.create_pose_stamped(2.35, 2.69, 1.57),
            'table_1': self.create_pose_stamped(6.92, 1.92, 1.68),
            'table_2': self.create_pose_stamped(6.75, 5.18, 1.57),
            'table_3': self.create_pose_stamped(6.74, 7.99, 1.61),
        }
        
        # Order management
        self.order_queue = []
        self.current_order = None
        self.food_loaded = False
        self.current_location = 'home'
        self.confirmation_timer = None
        self.confirmation_timeout = 30.0
        self.received_confirmation = False
        
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
        table_number = msg.data
        if table_number not in self.order_queue and self.current_order != table_number:
            self.order_queue.append(table_number)
            self.publish_status(f"ORDER_QUEUED: Table {table_number}")
            
            if self.current_location == 'home' and not self.current_order:
                self.process_next_order()

    def process_next_order(self):
        if not self.order_queue:
            if self.current_location != 'home':
                self.return_home()
            return
            
        self.current_order = self.order_queue.pop(0)
        self.publish_status(f"PROCESSING_ORDER: Table {self.current_order}")
        self.go_to_kitchen()

    def go_to_kitchen(self):
        self.cancel_timer_safe()
        self.publish_status("MOVING_TO_KITCHEN")
        
        try:
            self.navigator.goToPose(self.locations['kitchen'])
            while not self.navigator.isTaskComplete():
                pass
                
            self.current_location = 'kitchen'
            self.wait_for_confirmation('kitchen')
            
        except Exception as e:
            self.get_logger().error(f"Kitchen navigation error: {str(e)}")
            self.return_home()

    def go_to_table(self):
        self.cancel_timer_safe()
        self.publish_status(f"MOVING_TO_TABLE_{self.current_order}")
        
        try:
            self.navigator.goToPose(self.locations[f'table_{self.current_order}'])
            while not self.navigator.isTaskComplete():
                pass
                
            self.current_location = f'table_{self.current_order}'
            self.wait_for_confirmation('table')
            
        except Exception as e:
            self.get_logger().error(f"Table navigation error: {str(e)}")
            self.return_to_kitchen()

    def wait_for_confirmation(self, location):
        self.cancel_timer_safe()
        self.publish_status(f"WAITING_AT_{location.upper()}_FOR_CONFIRMATION")
        self.confirmation_pub.publish(String(data=location))
        
        self.received_confirmation = False
        self.confirmation_timer = self.create_timer(
            self.confirmation_timeout, 
            lambda: self.on_confirmation_timeout(location)
        )
        
        #while not self.received_confirmation and rclpy.ok():
            #rclpy.spin_once(self, timeout_sec=0.1)
            
        #self.cancel_timer_safe()

    def handle_confirmation(self, msg):
        self.received_confirmation = msg.data
        if msg.data:
            self.publish_status("CONFIRMATION_RECEIVED")
            if self.current_location == 'kitchen':
                self.food_loaded = True
                self.go_to_table()
            else:  # at table
                self.complete_delivery()
        else:
            self.publish_status("CONFIRMATION_DENIED")
            self.handle_confirmation_failure()

    def on_confirmation_timeout(self, location):
        self.publish_status(f"TIMEOUT_AT_{location.upper()}_NO_CONFIRMATION")
        self.handle_confirmation_failure()

    def handle_confirmation_failure(self):
        if self.current_location == 'kitchen':
            self.return_home()
        else:  # at table
            self.return_to_kitchen()

    def complete_delivery(self):
        self.publish_status(f"DELIVERY_COMPLETE: Table {self.current_order}")
        self.current_order = None
        self.food_loaded = False
        self.process_next_order()   

    def return_to_kitchen(self):
        self.cancel_timer_safe()
        self.publish_status("RETURNING_TO_KITCHEN")
        
        try:
            self.navigator.goToPose(self.locations['kitchen'])
            while not self.navigator.isTaskComplete():
                pass
                
            self.current_location = 'kitchen'
            self.process_next_order()
            
        except Exception as e:
            self.get_logger().error(f"Kitchen return error: {str(e)}")
            self.return_home()

    def return_home(self):
        self.cancel_timer_safe()
        self.publish_status("RETURNING_HOME")
        
        try:
            self.navigator.goToPose(self.locations['home'])
            while not self.navigator.isTaskComplete():
                pass
                
            self.current_location = 'home'
            self.publish_status("READY_AT_HOME")
            
        except Exception as e:
            self.get_logger().error(f"Home return error: {str(e)}")

    def cancel_timer_safe(self):
        if self.confirmation_timer:
            self.confirmation_timer.cancel()
            self.confirmation_timer = None

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    delivery_robot = DeliveryRobot()
    
    try:
        rclpy.spin(delivery_robot)
    except KeyboardInterrupt:
        pass
    finally:
        delivery_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()