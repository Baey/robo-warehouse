import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class OmniScanSubscriber(Node):
    """
    Node that subscribes to omni scan LaserScan data and processes it.
    """

    def __init__(self):
        """
        Initialize the node and set up the subscriber.
        """
        super().__init__('OmniScanSubscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan',  # LaserScan topic
            self.laser_scan_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def laser_scan_callback(self, msg):
        """
        Callback function to process the received LaserScan message.
        """
        # Log basic info about the LaserScan
        self.get_logger().info(f"Received LaserScan with {len(msg.ranges)} ranges")

        # Example: Find the closest object
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)
        min_angle = msg.angle_min + min_index * msg.angle_increment

        self.get_logger().info(f"Closest object at {min_range:.2f} meters, angle {math.degrees(min_angle):.2f}°")

def main(args=None):
    """
    Main function to run the OmniScanSubscriber node.
    """
    rclpy.init(args=args)
    node = OmniScanSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
