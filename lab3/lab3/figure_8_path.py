import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class Figure8Path(Node):
    def __init__(self):
        super().__init__('figure_8_path')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Параметри
        self.linear_speed = 0.4 
        self.angular_speed = 0.8
        
        # Час кола з поправкою на інерцію (приблизно 11-12 секунд разом із запасом)
        # Збільшуємо запас до 4.5, щоб кола замикалися
        self.circle_duration = ((2 * 3.14159) / self.angular_speed) + 4.5
        
        # Даємо системі час на встановлення зв'язку
        time.sleep(2.0)
        self.run_figure_8()

    def run_figure_8(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link' 

        # 1. Коло вліво
        self.get_logger().info(f"Moving left circle for {self.circle_duration:.2f}s...")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        time.sleep(self.circle_duration)

        # Коротке гальмування між колами для стабільності
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)
        time.sleep(0.5)

        # 2. Коло вправо
        self.get_logger().info(f"Moving right circle for {self.circle_duration:.2f}s...")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = -self.angular_speed 
        self.publisher_.publish(msg)
        time.sleep(self.circle_duration)

        # 3. Зупинка
        self.get_logger().info("Figure-8 completed. Stopping.")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Figure8Path()
    node.destroy_node()
    rclpy.shutdown()