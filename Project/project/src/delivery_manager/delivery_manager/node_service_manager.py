import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from custom_interfaces.srv import StartDelivery  # You'll need to create this interface

class ServiceManagerNode(Node):
    def __init__(self):
        super().__init__('service_manager_node')
        
        # Services
        self.start_service = self.create_service(
            StartDelivery,
            '/start_delivery',
            self.handle_start_delivery
        )
        
        self.end_service = self.create_service(
            Trigger,
            '/end_delivery',
            self.handle_end_delivery
        )
        
        self.get_logger().info('Service Manager Node initialized')

    def handle_start_delivery(self, request, response):
        self.get_logger().info(f'Starting delivery to: {request.destination}')
        response.success = True
        response.message = 'Delivery started'
        return response

    def handle_end_delivery(self, request, response):
        self.get_logger().info('Ending delivery')
        response.success = True
        response.message = 'Delivery completed'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
