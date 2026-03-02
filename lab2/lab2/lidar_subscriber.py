import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Логіка обробки даних лідара
        min_distance = min(msg.ranges)
        self.get_logger().info(f'Мінімальна відстань: {min_distance:.2f} м')

def main(args=None):  # Ось ця функція має бути обов'язково!
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()