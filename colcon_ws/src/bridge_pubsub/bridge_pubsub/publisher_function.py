# Copied from ROS 2 Humble Docs
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class BridgePublisher(Node):

    def __init__(self):
        super().__init__('bridge_publisher')
        self.publisher_ = self.create_publisher(String, 'testtopic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Example Data: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

# Extracted Abstraction for Readability
def main(args=None):
    rclpy.init(args=args)

    bridge_publisher = BridgePublisher()

    rclpy.spin(bridge_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bridge_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()