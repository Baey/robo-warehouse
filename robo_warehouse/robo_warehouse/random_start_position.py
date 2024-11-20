import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
import random

class RandomPositionInitializer(Node):
    """
    Node that initializes a random position for the robot in Gazebo.

    To use the bridge, run the following command:
    ros2 run ros_gz_bridge parameter_bridge /gazebo/set_model_state@gazebo_msgs/srv/SetModelState@gz.msgs.SetModelState
    """

    def __init__(self):
        """
        Initialize the RandomPositionInitializer node.
        
        This node will request a random position for the robot in Gazebo.
        It calls the 'set_model_state' service to set the robot's pose.
        """
        super().__init__('random_position_initializer')
        
        # Create a client to call the 'set_model_state' service in Gazebo
        self.client_ = self.create_client(SetModelState, '/gazebo/set_model_state')

        # Wait for the service to be available
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_model_state service to become available...')

        # Create a random position
        self.set_random_position()

    def set_random_position(self):
        """
        Set a random position for the robot by calling the 'set_model_state' service.
        The robot will be positioned at a random (x, y) coordinate, and orientation will also be random.
        """
        # Create a request to the service
        request = SetModelState.Request()

        # Set random position and orientation for the robot
        request.model_state.model_name = 'tugbot'
        
        # Random position within a certain range
        request.model_state.pose.position.x = random.uniform(-5.0, 5.0)  # Random X between -5 and 5 meters
        request.model_state.pose.position.y = random.uniform(-5.0, 5.0)  # Random Y between -5 and 5 meters
        request.model_state.pose.position.z = 0.0  # Keeping Z at 0 (ground level)

        # Random orientation (quaternion)
        request.model_state.pose.orientation = Quaternion(
            x=random.uniform(-1.0, 1.0),
            y=random.uniform(-1.0, 1.0),
            z=random.uniform(-1.0, 1.0),
            w=random.uniform(-1.0, 1.0)
        )

        # Call the service to set the position and orientation
        future = self.client_.call_async(request)
        future.add_done_callback(self.set_position_callback)

    def set_position_callback(self, future):
        """
        Callback that handles the result of the service call.
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Successfully set the random position of the robot.')
            else:
                self.get_logger().error('Failed to set the random position of the robot.')
        except Exception as e:
            self.get_logger().error('Service call failed: %r' % e)

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
