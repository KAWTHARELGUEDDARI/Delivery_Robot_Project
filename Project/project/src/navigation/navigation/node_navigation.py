import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        # Publishers
        self.position_publisher = self.create_publisher(
            PoseStamped,
            '/robot_position',
            10
        )
        
        # Current position
        self.current_position = PoseStamped()
        
        # Timer for position updates
        self.create_timer(0.1, self.update_position)
        
        self.get_logger().info('Navigation Node initialized')

    def update_position(self):
        # Simulate position updates
        self.current_position.header.stamp = self.get_clock().now().to_msg()
        self.position_publisher.publish(self.current_position)

    def set_target_position(self, x, y):
        # Set new target position
        self.target_x = x
        self.target_y = y
        self.get_logger().info(f'New target set: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()