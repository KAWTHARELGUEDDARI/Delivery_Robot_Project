import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_interfaces.action import DeliveryStatus  # You'll need to create this interface
import time

class ActionManagerNode(Node):
    def __init__(self):
        super().__init__('action_manager_node')
        
        # Action Server
        self._action_server = ActionServer(
            self,
            DeliveryStatus,
            '/delivery_status',
            self.execute_callback
        )
        
        self.get_logger().info('Action Manager Node initialized')

    async def execute_callback(self, goal_handle):
        feedback_msg = DeliveryStatus.Feedback()
        
        # Simulate delivery progress
        for i in range(0, 101, 10):
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return DeliveryStatus.Result()
                
            feedback_msg.progress = i
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)
        
        goal_handle.succeed()
        
        result = DeliveryStatus.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ActionManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
