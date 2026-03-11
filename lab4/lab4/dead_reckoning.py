import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry, Path

"""Dead reckoning - STUDENT TASK.
Integrate /cmd_vel to estimate pose; compare with Gazebo ground truth (/odom).
"""

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("ground_truth_topic", "/odom")
        self.declare_parameter("path_dr_topic", "/path_dr")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("max_poses", 2000)

        cmd_topic = self.get_parameter("cmd_vel_topic").value
        gt_topic = self.get_parameter("ground_truth_topic").value
        path_topic = self.get_parameter("path_dr_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.max_poses = int(self.get_parameter("max_poses").value)

        self.create_subscription(TwistStamped, cmd_topic, self.cmd_callback, 10)
        self.create_subscription(Odometry, gt_topic, self.gt_callback, 10)
        self.pub_path = self.create_publisher(Path, path_topic, 10)

        # Стан робота: x, y та yaw (кут повороту)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Час останнього повідомлення для розрахунку dt
        self.last_time = None
        
        # Змінна для зберігання істинної позиції
        self.gt_pose = None

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id

    def cmd_callback(self, msg: TwistStamped):
        # 1. Розрахунок дельти часу (dt)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # 2. Отримання швидкостей
        v = msg.twist.linear.x
        w = msg.twist.angular.z

        # 3. МАТЕМАТИЧНА МОДЕЛЬ
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        # 4. Створення повідомлення для візуалізації
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        
        # Кватерніон для орієнтації
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > self.max_poses:
            self.path_msg.poses.pop(0)

        self.path_msg.header.stamp = msg.header.stamp
        self.pub_path.publish(self.path_msg)

        # 5. Вивід похибки
        if self.gt_pose is not None:
            error = math.sqrt((self.x - self.gt_pose.x)**2 + (self.y - self.gt_pose.y)**2)
            self.get_logger().info(f"DR Error: {error:.4f} m", throttle_duration_sec=1.0)

    def gt_callback(self, msg: Odometry):
        self.gt_pose = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()