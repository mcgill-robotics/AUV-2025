# Copied from ROS 2 Humble Docs
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BridgeSubscriber(Node):

    def __init__(self):
        super().__init__('bridge_subscriber')
        self.subscription = self.create_subscription(
            String,
            'testtopic',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info("Listener Data: %s" % msg.data)

# Extracted Abstraction for Readability
def main(args=None):
    rclpy.init(args=args)

    bridge_subscriber = BridgeSubscriber()

    rclpy.spin(bridge_subscriber)

    # Destroy the node explicitly
    
    bridge_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()