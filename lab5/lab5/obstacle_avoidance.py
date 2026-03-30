import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.declare_parameter("goal_x", 3.0)
        self.declare_parameter("goal_y", 3.0)

        # 1. Створення видавця та підписників
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Змінні стану
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.scan_ranges = []

        # Таймер для циклу керування (10 Гц)
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Конвертація кватєрніона в Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    def control_loop(self):
        if not self.scan_ranges:
            return

        goal_x = self.get_parameter("goal_x").value
        goal_y = self.get_parameter("goal_y").value

        # --- Притягальна сила ---
        dx = goal_x - self.robot_x
        dy = goal_y - self.robot_y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        
        f_att_x = dx / dist_to_goal if dist_to_goal > 0.1 else 0.0
        f_att_y = dy / dist_to_goal if dist_to_goal > 0.1 else 0.0

        # --- Відштовхувальна сила ---
        f_rep_x = 0.0
        f_rep_y = 0.0
        eta = 0.5  # Коефіцієнт відштовхування
        rho_0 = 1.0 # Радіус впливу перешкод

        for i, distance in enumerate(self.scan_ranges):
            if 0.1 < distance < rho_0:
                # Кут променя i відносно орієнтації робота
                angle = i * (2 * math.pi / len(self.scan_ranges)) + self.robot_yaw
                # Формула сили: eta * (1/dist - 1/rho_0) * (1/dist^2)
                rep_mag = eta * (1.0/distance - 1.0/rho_0) / (distance**2)
                f_rep_x -= rep_mag * math.cos(angle)
                f_rep_y -= rep_mag * math.sin(angle)

        # --- Результуючий вектор ---
        total_x = f_att_x + f_rep_x
        total_y = f_att_y + f_rep_y
        
        target_yaw = math.atan2(total_y, total_x)
        angle_error = target_yaw - self.robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error)) # Нормалізація

   # --- Команда на рух ---
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if dist_to_goal < 0.2:
            self.get_logger().info("Goal Reached!")
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        else:
            msg.twist.linear.x = min(0.2, dist_to_goal)
            msg.twist.angular.z = 1.2 * angle_error
        
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()