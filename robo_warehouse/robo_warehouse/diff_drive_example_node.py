import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TugbotDiffDriveController(Node):
    """
    Node that publishes Twist messages to control a tugbot.

    To set up the bridge, run the following command:
    ros2 run ros_gz_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
    """

    def __init__(self):
        """
        Initialize the TugbotDiffDriveController node.
        
        Creates a publisher for Twist messages on the '/model/tugbot/cmd_vel' topic
        and sets up a timer to call the timer_callback method at regular intervals.
        """
        super().__init__('tugbot_diff_drive_controller')
        self.publisher_ = self.create_publisher(Twist, '/model/tugbot/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function that publishes Twist messages at regular intervals.
        
        This function is called by the timer and publishes a Twist message with
        a fixed linear and angular velocity to the '/model/tugbot/cmd_vel' topic.
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 10.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    """
    Main function to initialize and spin the TugbotDiffDriveController node.
    
    :param args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    node = TugbotDiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
