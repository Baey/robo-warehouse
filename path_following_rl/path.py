import random
import math

def generate_path(start_x: float, start_y: float, num_points: int = 10, width: int = 800, height: int = 600) -> list:
    """
    Generates a random path starting from the given position, ensuring it stays within the window.

    Args:
        start_x (float): The starting x-coordinate of the path.
        start_y (float): The starting y-coordinate of the path.
        num_points (int): The number of points in the path.
        width (int): The width of the window.
        height (int): The height of the window.

    Returns:
        list: A list of (x, y) tuples representing the path.
    """
    path = [(start_x, start_y)]
    angle = random.uniform(0, 2 * math.pi)
    step_size = 50
    center_x, center_y = width / 2, height / 2

    for _ in range(num_points - 1):
        if path[-1][0] < 50 or path[-1][0] > width - 50 or path[-1][1] < 50 or path[-1][1] > height - 50:
            angle_towards_center = math.atan2(center_y - path[-1][1], center_x - path[-1][0])
            angle = (angle + angle_towards_center) / 2  # Subtly direct towards center
        else:
            angle += random.uniform(-math.pi / 8, math.pi / 8)  # Smaller random change in angle

        next_x = min(max(path[-1][0] + step_size * math.cos(angle), 0), width)
        next_y = min(max(path[-1][1] + step_size * math.sin(angle), 0), height)
        path.append((next_x, next_y))
    return path
