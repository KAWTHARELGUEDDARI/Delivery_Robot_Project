import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA

class RvizVisualizationNode(Node):
    def __init__(self):
        super().__init__('rviz_visualization_node')
        
        # Publishers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_markers',
            10
        )
        
        # Subscribers
        self.position_subscription = self.create_subscription(
            PoseStamped,
            '/robot_position',
            self.position_callback,
            10
        )
        
        self.markers = MarkerArray()
        self.get_logger().info('Rviz Visualization Node initialized')

    def position_callback(self, msg):
        # Create robot marker
        marker = Marker()
        marker.header = msg.header
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        self.markers.markers = [marker]
        self.marker_publisher.publish(self.markers)

def main(args=None):
    rclpy.init(args=args)
    node = RvizVisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
