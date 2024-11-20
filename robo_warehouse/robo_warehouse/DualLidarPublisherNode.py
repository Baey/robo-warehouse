import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class DualLidarPublisherNode(Node):
    """
    Node that publishes LiDAR data from two sensors: scan_front and scan_back.
    """

    def __init__(self):
        """
        Initialize the DualLidarPublisherNode.
        
        Creates publishers for LaserScan messages on the '/scan_front' and '/scan_back' topics.
        """
        super().__init__('dual_lidar_publisher_node')
        self.front_publisher_ = self.create_publisher(LaserScan, '/scan_front/sensor/scan_front/scan/points', 10)
        self.back_publisher_ = self.create_publisher(LaserScan, '/scan_back/sensor/scan_front/scan/points', 10)
        timer_period = 0.1  # Publish every 0.1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function to publish mock data for the two LiDAR sensors.
        """
        front_scan = self.create_mock_scan("front")
        back_scan = self.create_mock_scan("back")
        
        self.front_publisher_.publish(front_scan)
        self.back_publisher_.publish(back_scan)
        self.get_logger().info('Publishing data for scan_front and scan_back')

    def create_mock_scan(self, sensor_name):
        """
        Generate a mock LaserScan message for a given sensor.
        """
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = f"{sensor_name}_frame"
        scan.angle_min = -1.57  # -90 degrees
        scan.angle_max = 1.57   # +90 degrees
        scan.angle_increment = 0.01  # Approx. 1 degree increment
        scan.range_min = 0.2
        scan.range_max = 30.0
        
        # Mock data: ranges with sinusoidal variation
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [5.0 + 0.5 * (1 if sensor_name == "front" else -1) for _ in range(num_readings)]
        scan.intensities = [1.0] * num_readings
        return scan

def main(args=None):
    """
    Main function to initialize and spin the DualLidarPublisherNode.
    """
    rclpy.init(args=args)
    node = DualLidarPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
