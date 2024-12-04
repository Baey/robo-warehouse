import math
import pygame
import random
import numpy as np

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
    Generates a smoother random path starting from the given position.
    """
    path = [(start_x, start_y)]
    angle = random.uniform(0, 2 * math.pi)
    
    # Define boundary margins
    margin = step_size * 2
    
    for _ in range(num_points - 1):
        current_x, current_y = path[-1]
        
        # Calculate distance to borders
        dist_to_left = current_x
        dist_to_right = width - current_x
        dist_to_top = current_y
        dist_to_bottom = height - current_y
        
        # Calculate center direction
        center_x, center_y = width / 2, height / 2
        angle_to_center = math.atan2(center_y - current_y, center_x - current_x)
        
        # Adjust angle based on proximity to borders
        border_influence = 0.0
        if (dist_to_left < margin or dist_to_right < margin or 
            dist_to_top < margin or dist_to_bottom < margin):
            border_influence = 0.7
        
        # Smoothly blend between random walk and center-seeking behavior
        new_angle = angle + random.uniform(-math.pi/4, math.pi/4)
        blended_angle = (1 - border_influence) * new_angle + border_influence * angle_to_center
        
        # Update angle with smooth transition
        angle = blended_angle
        
        # Calculate next point
        next_x = current_x + step_size * math.cos(angle)
        next_y = current_y + step_size * math.sin(angle)
        
        # Ensure point stays within bounds
        next_x = max(margin/2, min(width - margin/2, next_x))
        next_y = max(margin/2, min(height - margin/2, next_y))
        
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
    """
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)  # Fixed p2[1] - p2[1] to p2[1] - p1[1]

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
    x = width // 2 + random.randint(-width // 4, width // 4)
    y = height // 2 + random.randint(-height // 4, height // 4)
    theta = random.uniform(0, 2 * math.pi)
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

def calculate_reward(last_distance: float, dist_to_next_point: float, theta: float, x: float, y: float, next_point: tuple, prev_point: tuple, margin: float = 20) -> tuple:
    # Calculate potential reward
    potential_reward = -0.1
    if dist_to_next_point < last_distance:
        potential_reward = last_distance - dist_to_next_point

    # Calculate heading reward
    desired_theta = math.atan2(
        next_point[1] - y,
        next_point[0] - x,
    )
    angle_diff = abs(desired_theta - theta) % (2 * math.pi)
    if angle_diff > math.pi:
        angle_diff = 2.0 * math.pi - angle_diff
    heading_reward = math.cos(angle_diff)

    # Calculate checkpoint reward
    checkpoint_reward = 0.0
    if dist_to_next_point < margin:
        checkpoint_reward = 100.0

    # Calculate belt reward
    path_vector = np.array(next_point) - np.array(prev_point)
    path_length = np.linalg.norm(path_vector)
    path_unit_vector = path_vector / path_length
    
    robot_vector = np.array([x, y]) - np.array(prev_point)
    projection_length = np.dot(robot_vector, path_unit_vector)
    projection_point = np.array(prev_point) + projection_length * path_unit_vector
    distance_to_path = np.linalg.norm(np.array([x, y]) - projection_point)
    belt_width = 20.0
    belt_reward = 1.0 - (distance_to_path / belt_width) if distance_to_path < belt_width else 0.0

    # Calculate total reward
    total_reward = (
        potential_reward * 1.0 +
        checkpoint_reward * 2.0 + 
        heading_reward * 0.1 +
        belt_reward * 0.5 - 0.1
    )

    return total_reward, potential_reward, heading_reward, checkpoint_reward, belt_reward

def display_reward(screen, reward_info: tuple) -> None:
    font = pygame.font.Font(None, 24)
    total_reward, potential_reward, heading_reward, checkpoint_reward, belt_reward = reward_info
    
    rewards_text = {
        "potential_reward": potential_reward,
        "heading_reward": heading_reward,
        "checkpoint_reward": checkpoint_reward,
        "belt_reward": belt_reward,
        "total_reward": total_reward
    }
    
    y_offset = 100
    for key, value in rewards_text.items():
        text = font.render(f"{key}: {value:.2f}", True, BLACK)
        screen.blit(text, (20, y_offset))
        y_offset += 30

def display_observations(screen, x: float, y: float, theta: float, v: float, omega: float, next_point: tuple, path: list, next_point_index: int) -> None:
    # Calculate observations for next point
    desired_theta = math.atan2(
        next_point[1] - y,
        next_point[0] - x,
    )
    rotation_diff = (desired_theta - theta) % (2 * math.pi)
    if rotation_diff > math.pi:
        rotation_diff = rotation_diff - 2 * math.pi
        rotation_diff = rotation_diff / math.pi
        
    dist = distance((x, y), next_point)
    normalized_distance = dist / 450

    # Calculate observations for next-next point
    next_next_point_index = next_point_index + 1 if next_point_index + 1 < len(path) else next_point_index
    next_next_point = path[next_next_point_index]
    
    next_next_desired_theta = math.atan2(
        next_next_point[1] - y,
        next_next_point[0] - x,
    )
    next_next_rotation_diff = (next_next_desired_theta - theta) % (2 * math.pi)
    if next_next_rotation_diff > math.pi:
        next_next_rotation_diff = next_next_rotation_diff - 2 * math.pi
        next_next_rotation_diff = next_next_rotation_diff / math.pi
        
    next_next_dist = distance((x, y), next_next_point)
    next_next_normalized_distance = next_next_dist / 450
    
    normalized_lin_vel = v / MAX_SPEED
    normalized_ang_vel = omega / MAX_OMEGA
    
    # Display observations
    font = pygame.font.Font(None, 24)
    y_offset = 300
    
    labels = [
        'Linear Velocity',
        'Angular Velocity',
        'Next Point Distance',
        'Next Point Rotation',
        'Next-Next Distance',
        'Next-Next Rotation'
    ]
    values = [
        normalized_lin_vel,
        normalized_ang_vel,
        normalized_distance,
        rotation_diff,
        next_next_normalized_distance,
        next_next_rotation_diff
    ]
    
    text = font.render("Observations:", True, BLACK)
    screen.blit(text, (20, y_offset))
    y_offset += 30
    
    for label, value in zip(labels, values):
        text = font.render(f"{label}: {value:.2f}", True, BLACK)
        screen.blit(text, (20, y_offset))
        y_offset += 25

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
    margin = 20  # Add margin constant to match gym env

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
        current_distance = distance((x, y), path[next_point_index])
        if current_distance < margin:  # Use margin constant
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
            path[next_point_index],
            path[next_point_index - 1],  # Pass previous point
            margin  # Use margin constant
        )
        last_distance = current_distance

        screen.fill(WHITE)
        draw_path(screen, path, next_point_index)  # Draw the path with the next point marked
        draw_robot(screen, x, y, theta)
        display_info(screen, x, y, path[next_point_index], current_distance)  # Draw the info
        display_reward(screen, reward_info)  # Add reward display
        display_observations(screen, x, y, theta, v, omega, path[next_point_index], path, next_point_index)  # Updated

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
