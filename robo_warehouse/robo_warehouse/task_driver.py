import rclpy
import getch
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class Nav2GoalController(Node):
    """
    Node that publishes PoseStamped messages to control a robot using Nav2.

    This node allows the user to input goal coordinates via the console.
    The robot will move to the specified goals based on the selected task.
    """

    def __init__(self):
        """
        Initialize the Nav2GoalController node.

        Creates a publisher for PoseStamped messages on the '/goal_pose' topic
        and subscribes to the '/odom' topic to monitor the robot's position.
        """
        super().__init__('nav2_goal_controller')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/model/tugbot/odometry',
            self.odom_callback,
            10
        )
        self.goal = PoseStamped()
        self.current_position = (0.0, 0.0)
        self.goal_tolerance = 1.0  # Distance threshold to consider goal reached
        self.goal_active = False
        self.goal_queue = []
        self.home_position = (0.0, 0.0)
        self.get_logger().info('Nav2 Goal Controller started.')
        self.run_task_selector()

    def publish_goal(self, x, y, z=0.0, w=1.0):
        """
        Publish a PoseStamped message with the specified coordinates.

        :param x: X-coordinate of the goal.
        :param y: Y-coordinate of the goal.
        :param z: Z-orientation of the goal (default is 0.0).
        :param w: W-orientation of the goal (default is 1.0).
        """
        self.goal.header.frame_id = 'map'
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.position.z = 0.0
        self.goal.pose.orientation.z = z
        self.goal.pose.orientation.w = w

        self.publisher_.publish(self.goal)
        self.goal_active = True
        self.get_logger().info(f'Published goal: x={x}, y={y}')

    def has_reached_goal(self, x, y):
        """
        Check if the robot has reached the goal within a tolerance.

        :param x: X-coordinate of the goal.
        :param y: Y-coordinate of the goal.
        :return: True if the robot is within the goal tolerance, False otherwise.
        """
        current_x, current_y = self.current_position
        distance = math.sqrt((x - current_x)**2 + (y - current_y)**2)
        return distance <= self.goal_tolerance

    def odom_callback(self, msg):
        """
        Callback function to update the robot's current position based on odometry
        and check if the robot has reached the goal.
        """
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

        if self.goal_active:
            x_goal, y_goal = self.goal.pose.position.x, self.goal.pose.position.y
            if self.has_reached_goal(x_goal, y_goal):
                self.goal_active = False
                self.get_logger().info(f'Reached goal at ({x_goal}, {y_goal})')
                if self.goal_queue:
                    next_goal = self.goal_queue.pop(0)
                    self.publish_goal(*next_goal)

    def task_single_run(self):
        """
        Task: Accept user input for multiple goal coordinates and move to those goals sequentially.
        """
        try:
            print("Enter multiple goals (x y), one per line. Type 'done' when finished:")
            while True:
                input_coords = input().strip()
                if input_coords.lower() == 'done':
                    break
                try:
                    x, y = map(float, input_coords.split())
                    self.goal_queue.append((x, y))
                except ValueError:
                    self.get_logger().error('Invalid input. Please enter numeric values in the format "x y".')

            if not self.goal_queue:
                self.get_logger().info('No goals entered. Returning to menu.')
                return

            # Start with the first goal
            first_goal = self.goal_queue.pop(0)
            self.publish_goal(*first_goal)

            # Process the remaining goals
            while self.goal_active or self.goal_queue:
                rclpy.spin_once(self)

            self.get_logger().info('All goals have been visited. Task completed.')

        except KeyboardInterrupt:
            self.get_logger().info('Shutting down task.')

    def task_two_points(self):
        """
        Task: Travel between two points repeatedly.
        """
        try:
            print("Enter first point (x y):")
            x1, y1 = map(float, input().strip().split())
            print("Enter second point (x y):")
            x2, y2 = map(float, input().strip().split())

            while True:
                self.publish_goal(x1, y1)
                while not self.has_reached_goal(x1, y1):
                    rclpy.spin_once(self)

                self.publish_goal(x2, y2)
                while not self.has_reached_goal(x2, y2):
                    rclpy.spin_once(self)

        except ValueError:
            self.get_logger().error('Invalid input. Please enter numeric values in the format "x y".')
        except KeyboardInterrupt:
            self.get_logger().info('Exiting two points task.')

    def task_return_to_base(self):
        """
        Task: Return to the base location.
        """
        self.get_logger().info('Returning to base. Base coordinates: ({}, {})'.format(*self.home_position))
        self.publish_goal(*self.home_position)
        while not self.has_reached_goal(*self.home_position):
            rclpy.spin_once(self)
        self.get_logger().info('Returned to base.')

    def run_task_selector(self):
        """
        Allow the user to select and run a specific task.
        """
        while True:
            print("Select a task:")
            print("1: Visit multiple points once")
            print("2: Travel between two points")
            print("3: Return to base")
            print("4: Exit")

            choice = input("Enter your choice: ").strip()
            if choice == '1':
                self.task_single_run()
            elif choice == '2':
                self.task_two_points()
            elif choice == '3':
                self.task_return_to_base()
            elif choice == '4':
                self.get_logger().info('Exiting Nav2 Goal Controller.')
                break
            else:
                self.get_logger().info('Invalid choice. Please select a valid option.')


def main(args=None):
    """
    Main function to initialize and spin the Nav2GoalController node.

    :param args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    node = Nav2GoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
