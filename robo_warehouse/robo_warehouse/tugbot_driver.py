import rclpy
import getch
import threading
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
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cmd = Twist()
        self.teleop = False
        self.get_logger().info('Starting Tugbot Controller')
        self.get_logger().info('Press "t" to toggle teleop mode')
        self.get_logger().info('In teleop mode, use "w", "s", "a", "d" to move and space to stop')
        self.teleop_thread = threading.Thread(target=self.teleop_listener)
        self.teleop_thread.daemon = True
        self.teleop_thread.start()

    def teleop_listener(self):
        """
        Thread function to listen for keyboard inputs at a higher frequency.
        """
        while rclpy.ok():
            key = getch.getch()
            if key == 't':
                self.teleop = not self.teleop
                if self.teleop:
                    self.get_logger().info('Switching into teleop mode')
                else:
                    self.get_logger().info('Switching into automatic mode')
            if self.teleop:
                self.process_teleop_key(key)

    def process_teleop_key(self, key):
        """
        Process the key input for teleop mode.
        """
        if key == 'w':
            self.cmd.linear.x += 0.5
            if self.cmd.linear.x > 1.0:
                self.cmd.linear.x = 1.0
            self.get_logger().info(f'(w) Forward {self.cmd.linear.x}')
        elif key == 's':
            self.cmd.linear.x += -0.5
            if self.cmd.linear.x < -1.0:
                self.cmd.linear.x = -1.0
            self.get_logger().info(f'(s) Backward {self.cmd.linear.x}')
        elif key == 'a':
            self.cmd.angular.z += 0.5
            if self.cmd.angular.z > 1.0:
                self.cmd.angular.z = 1.0
            self.get_logger().info(f'(a) Left {self.cmd.angular.z}')
        elif key == 'd':
            self.cmd.angular.z += -0.5
            if self.cmd.angular.z < -1.0:
                self.cmd.angular.z = -1.0
            self.get_logger().info(f'(d) Right {self.cmd.angular.z}')
        elif key == ' ':
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.get_logger().info('(space) Stop')

    def timer_callback(self):
        """
        Callback function that publishes Twist messages at regular intervals.
        
        This function is called by the timer and publishes a Twist message with
        a fixed linear and angular velocity to the '/model/tugbot/cmd_vel' topic.
        """
        if not self.teleop:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 10.0

        self.publisher_.publish(self.cmd)

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
