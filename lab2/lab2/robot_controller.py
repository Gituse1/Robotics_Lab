import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = Twist()
        # ЛОГІКА РУХУ:
        msg.linear.x = 0.5  # Швидкість вперед (м/с)
        msg.angular.z = 0.3 * math.sin(self.counter * 0.1)  # Зигзаг
        
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()