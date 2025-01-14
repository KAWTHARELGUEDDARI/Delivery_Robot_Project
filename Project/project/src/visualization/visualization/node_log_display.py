import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_interfaces.action import DeliveryStatus

class LogDisplayNode(Node):
    def __init__(self):
        super().__init__('log_display_node')
        
        # Subscribers
        self.position_subscription = self.create_subscription(
            PoseStamped,
            '/robot_position',
            self.position_callback,
            10
        )
        
        self.delivery_subscription = self.create_subscription(
            DeliveryStatus,
            '/delivery_status',
            self.delivery_callback,
            10
        )
        
        self.get_logger().info('Log Display Node initialized')

    def position_callback(self, msg):
        self.get_logger().info(
            f'Robot Position - X: {msg.pose.position.x:.2f}, '
            f'Y: {msg.pose.position.y:.2f}'
        )

    def delivery_callback(self, msg):
        self.get_logger().info(f'Delivery Status: {msg.status}')

def main(args=None):
    rclpy.init(args=args)
    node = LogDisplayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()