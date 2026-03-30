import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time

class Figure8Path(Node):
    def __init__(self):
        super().__init__('figure_8_path')
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        
        
        self.circle_duration = ((2 * 3.14159) / self.angular_speed) + 4.5
        
        time.sleep(2.0)
        self.run_figure_8()

    def run_figure_8(self):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link' 

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
        self.linear_speed =0.2
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = -self.angular_speed 
        self.publisher_.publish(msg)
        time.sleep(self.circle_duration)

        self.get_logger().info(f"Moving left circle for {self.circle_duration:.2f}s...")
        self.linear_speed =0.1
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear_speed
        msg.twist.angular.z = self.angular_speed
        self.publisher_.publish(msg)
        time.sleep(self.circle_duration)

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