import math
import pygame
import random

# Constants
WIDTH, HEIGHT = 1200, 900
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
ROBOT_RADIUS = 20
ROBOT_WIDTH = 50
MAX_SPEED = 100
MAX_OMEGA = 1.0
base_reward: float = 0.0

def generate_path(start_x: float, start_y: float, num_points: int = 10, width: int = 800, height: int = 600, step_size: int = 100) -> list:
    """
    Generates a random path starting from the given position, ensuring it stays within the window.

    Args:
        start_x (float): The starting x-coordinate of the path.
        start_y (float): The starting y-coordinate of the path.
        num_points (int): The number of points in the path.
        width (int): The width of the window.
        height (int): The height of the window.
        step_size (int): The step size for each segment of the path.

    Returns:
        list: A list of (x, y) tuples representing the path.
    """
    path = [(start_x, start_y)]
    angle = random.uniform(0, 2 * math.pi)
    center_x, center_y = width / 2, height / 2

    for _ in range(num_points - 1):
        if path[-1][0] < step_size * 1.8 or path[-1][0] > width - step_size * 1.8 or path[-1][1] < step_size * 1.8 or path[-1][1] > height - step_size * 1.8:
            angle_towards_center = math.atan2(center_y - path[-1][1], center_x - path[-1][0])
            angle = (angle + angle_towards_center) / 2  # Subtly direct towards center
        else:
            angle += random.uniform(-math.pi / 8, math.pi / 8)  # Smaller random change in angle

        next_x = min(max(path[-1][0] + step_size * math.cos(angle), 0), width)
        next_y = min(max(path[-1][1] + step_size * math.sin(angle), 0), height)
        path.append((next_x, next_y))
    return path

def draw_robot(screen, x: float, y: float, theta: float) -> None:
    """
    Draws the robot on the screen.

    Args:
        x (float): The x-coordinate of the robot's center.
        y (float): The y-coordinate of the robot's center.
        theta (float): The orientation of the robot in radians.
    """
    front_x = x + ROBOT_RADIUS * math.cos(theta)
    front_y = y + ROBOT_RADIUS * math.sin(theta)
    rear_left_x = x - ROBOT_RADIUS * math.cos(theta) - ROBOT_WIDTH / 2 * math.sin(theta)
    rear_left_y = y - ROBOT_RADIUS * math.sin(theta) + ROBOT_WIDTH / 2 * math.cos(theta)
    rear_right_x = x - ROBOT_RADIUS * math.cos(theta) + ROBOT_WIDTH / 2 * math.sin(theta)
    rear_right_y = y - ROBOT_RADIUS * math.sin(theta) - ROBOT_WIDTH / 2 * math.cos(theta)

    pygame.draw.polygon(screen, BLUE, [(front_x, front_y), (rear_left_x, rear_left_y), (rear_right_x, rear_right_y)])
    pygame.draw.circle(screen, RED, (int(rear_left_x), int(rear_left_y)), 5)
    pygame.draw.circle(screen, RED, (int(rear_right_x), int(rear_right_y)), 5)
    pygame.draw.circle(screen, BLACK, (front_x, front_y), 5)

def draw_path(screen, path: list, next_point_index: int) -> None:
    """
    Draws the path on the screen and marks the starting, next, and ending points.

    Args:
        path (list): A list of (x, y) tuples representing the path.
        next_point_index (int): The index of the next point in the path.
    """
    for i in range(len(path) - 1):
        pygame.draw.line(screen, BLACK, path[i], path[i + 1], 2)
    pygame.draw.circle(screen, RED, (int(path[0][0]), int(path[0][1])), 5)  # Starting point
    pygame.draw.circle(screen, BLUE, (int(path[-1][0]), int(path[-1][1])), 5)  # Ending point
    if next_point_index < len(path):
        pygame.draw.circle(screen, GREEN, (int(path[next_point_index][0]), int(path[next_point_index][1])), 5)  # Next point

def display_info(screen, x: float, y: float, next_point: tuple, distance: float) -> None:
    """
    Draws text on the screen displaying the robot coordinates, next point coordinates, and distance between both.

    Args:
        screen: The pygame screen to draw on.
        x (float): The x-coordinate of the robot.
        y (float): The y-coordinate of the robot.
        next_point (tuple): The coordinates of the next point.
        distance (float): The distance between the robot and the next point.
    """
    font = pygame.font.Font(None, 36)
    text = font.render(f"Tugbot: ({x:.2f}, {y:.2f}) | Next Point: ({next_point[0]:.2f}, {next_point[1]:.2f}) | Distance: {distance:.2f}", True, BLACK)
    screen.blit(text, (20, 20))

def update_position(x: float, y: float, theta: float, v: float, omega: float, dt: float) -> tuple[float, float, float]:
    """
    Updates the position and orientation of the robot.

    Args:
        x (float): The current x-coordinate of the robot.
        y (float): The current y-coordinate of the robot.
        theta (float): The current orientation of the robot in radians.
        v (float): The linear velocity of the robot (px/s).
        omega (float): The angular velocity of the robot (rad/s).
        dt (float): The time step (s).

    Returns:
        tuple[float, float, float]: The updated x, y, and theta values.
    """
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += omega * dt
    theta %= 2 * math.pi
    return x, y, theta

def distance(p1: tuple, p2: tuple) -> float:
    """
    Calculates the Euclidean distance between two points.

    Args:
        p1 (tuple): The first point (x, y).
        p2 (tuple): The second point (x, y).

    Returns:
        float: The distance between the two points.
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def reset_environment(width: int, height: int, step_size: int = 100) -> tuple:
    """
    Resets the environment to the initial state.

    Args:
        width (int): The width of the window.
        height (int): The height of the window.
        step_size (int): The step size for each segment of the path.

    Returns:
        tuple: The initial state of the robot and the generated path.
    """
    x = width // 2
    y = height // 2
    theta = 0
    path = generate_path(x, y, width=width, height=height, step_size=step_size)
    return (x, y, theta, path)

def get_state(x: float, y: float, theta: float, v: float, omega: float, path: list, next_point_index: int) -> list:
    """
    Returns the current state of the robot.

    Args:
        x (float): The x-coordinate of the robot.
        y (float): The y-coordinate of the robot.
        theta (float): The orientation of the robot in radians.
        v (float): The linear velocity of the robot.
        omega (float): The angular velocity of the robot.
        path (list): The path the robot is following.
        next_point_index (int): The index of the next point in the path.

    Returns:
        list: The current state of the robot.
    """
    next_point = path[next_point_index]
    next_next_point = path[next_point_index + 1] if next_point_index + 1 < len(path) else next_point
    return [x, y, theta, v, omega, next_point[0], next_point[1], next_next_point[0], next_next_point[1]]

def calculate_reward(last_distance: float, dist_to_next_point: float, theta: float, x: float, y: float, next_point: tuple, margin: float = 20) -> float:
    global base_reward

    # Progress reward with momentum
    progress = last_distance - dist_to_next_point
    progress_reward = progress

    # Improved angle alignment reward
    desired_theta = math.atan2(
        next_point[1] - y,
        next_point[0] - x,
    )
    angle_diff = abs(desired_theta - theta) % (2 * math.pi)
    if angle_diff > math.pi:
        angle_diff = 2.0 * math.pi - angle_diff
    
    heading_reward = math.cos(angle_diff)

    # Checkpoint reward
    if dist_to_next_point < margin:
        base_reward += 100.0


    total_reward = (progress_reward * 10.0 + 
            dist_to_next_point * -0.2 +
            heading_reward * 0.5 + 
            base_reward)
    
    return total_reward, progress_reward, heading_reward, base_reward

def display_reward(screen, reward_info: tuple) -> None:
    font = pygame.font.Font(None, 36)
    total_reward, progress_reward, heading_reward, checkpoint_reward = reward_info
    rewards_text = [
        f"Total Reward: {total_reward:.2f}",
        f"Progress Reward: {progress_reward:.2f}",
        f"Heading Reward: {heading_reward:.2f}",
        f"Checkpoint Reward: {checkpoint_reward:.2f}",
        f"Base Reward: {base_reward:.2f}"
    ]
    
    for i, text in enumerate(rewards_text):
        surface = font.render(text, True, BLACK)
        screen.blit(surface, (20, 100 + i * 30))

def main():
    pygame.init()

    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Tugbot Differential Drive Environment")

    x = WIDTH // 2
    y = HEIGHT // 2
    theta = 0

    path = generate_path(x, y, width=WIDTH, height=HEIGHT, step_size=100)  # Generate the initial path

    running = True
    clock = pygame.time.Clock()
    v = 0
    omega = 0
    next_point_index = 1
    last_distance = None

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        keys = pygame.key.get_pressed()
        if keys[pygame.K_UP]:
            v = min(v + 10, MAX_SPEED)
        elif keys[pygame.K_DOWN]:
            v = max(v - 10, -MAX_SPEED)
        else:
            v *= 0.95  # Inertia for linear velocity

        if keys[pygame.K_LEFT]:
            omega = max(omega - 0.1, -MAX_OMEGA)
        elif keys[pygame.K_RIGHT]:
            omega = min(omega + 0.1, MAX_OMEGA)
        else:
            omega *= 0.9  # Inertia for angular velocity

        dt = clock.get_time() / 1000.0
        x, y, theta = update_position(x, y, theta, v, omega, dt)

        # Check if the robot is close to the next point in the path
        if distance((x, y), path[next_point_index]) < 20:
            if next_point_index == len(path) - 1:
                path = generate_path(x, y, width=WIDTH, height=HEIGHT, step_size=100)  # Generate a new path
                next_point_index = 1
            else:
                next_point_index += 1

        # Calculate rewards
        current_distance = distance((x, y), path[next_point_index])
        if last_distance is None:
            last_distance = current_distance
            
        reward_info = calculate_reward(
            last_distance, 
            current_distance, 
            theta, 
            x, 
            y, 
            path[next_point_index]
        )
        last_distance = current_distance

        screen.fill(WHITE)
        draw_path(screen, path, next_point_index)  # Draw the path with the next point marked
        draw_robot(screen, x, y, theta)
        display_info(screen, x, y, path[next_point_index], current_distance)  # Draw the info
        display_reward(screen, reward_info)  # Add reward display

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
