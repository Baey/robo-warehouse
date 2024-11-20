import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import random

class RandomPositionInitializer(Node):
    """
    Node that initializes a random position for the robot in Gazebo.

    To use the bridge, run the following command:
    ros2 run ros_gz_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist 
    """

    def __init__(self):
        """
        Initialize the RandomPositionInitializer node.
        
        This node will publish a random position for the robot in Gazebo.
        It publishes a Pose message to the '/model/tugbot/pose' topic.
        """
        super().__init__('random_position_initializer')
        
        # Create a publisher to publish the random pose of the robot
        self.publisher_ = self.create_publisher(Pose, '/model/tugbot/pose', 10)
        
        # Set a random position immediately upon initialization
        self.set_random_position()

    def set_random_position(self):
        """
        Set a random position for the robot by publishing a Pose message.
        The robot will be positioned at a random (x, y) coordinate, and orientation will also be random.
        """
        # Create a Pose message
        pose_msg = Pose()

        # Set random position within a certain range
        pose_msg.position.x = random.uniform(-5.0, 5.0)  # Random X between -5 and 5 meters
        pose_msg.position.y = random.uniform(-5.0, 5.0)  # Random Y between -5 and 5 meters
        pose_msg.position.z = 0.0  # Keeping Z at 0 (ground level)

        # Random orientation (quaternion)
        pose_msg.orientation = Quaternion(
            x=random.uniform(-1.0, 1.0),
            y=random.uniform(-1.0, 1.0),
            z=random.uniform(-1.0, 1.0),
            w=random.uniform(-1.0, 1.0)
        )

        # Publish the random position
        self.publisher_.publish(pose_msg)
        self.get_logger().info('Publishing random pose: %s' % str(pose_msg))

def main(args=None):
    """
    Main function to initialize and spin the RandomPositionInitializer node.
    
    :param args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    node = RandomPositionInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
