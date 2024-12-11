from collections import deque
import math
import random
import cv2
import numpy as np
import heapq
import math


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

class TugbotMapListener(Node):
    """
    Node that subscribes to the '/map' topic and logs or processes the received OccupancyGrid message.
    """

    def __init__(self):
        """
        Initialize the TugbotMapListener node.
        
        Creates a subscriber to listen to OccupancyGrid messages on the '/map' topic.
        """
        super().__init__('tugbot_map_listener')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """
        Callback function that processes the incoming OccupancyGrid message.
        
        Logs or processes the received map data.
        """
        self.get_logger().info('Received a new map with dimensions: %d x %d' % (msg.info.width, msg.info.height))

        # Extracting the grid data (occupancy values)
        grid_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # Visualizing the occupancy grid
        plt.imshow(grid_data, cmap='gray')
        plt.title('Occupancy Grid Map')
        plt.colorbar(label='Occupancy (0: free, 100: occupied, -1: unknown)')
        plt.show()

def main(args=None):
    """
    Main function to initialize and spin the TugbotMapListener node.
    
    :param args: Command line arguments passed to the node.
    """
    rclpy.init(args=args)
    node = TugbotMapListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()






















# def expand_obstacles(grid, a):
#     """
#     Rozszerza przeszkody w macierzy, aby uwzględnić szerokość obiektu.
#     """
#     rows, cols = len(grid), len(grid[0])
#     expanded_grid = [[0] * cols for _ in range(rows)]
#     margin = math.ceil(a / 2)

#     for r in range(rows):
#         for c in range(cols):
#             if grid[r][c] == 1:
#                 for dr in range(-margin, margin + 1):
#                     for dc in range(-margin, margin + 1):
#                         nr, nc = r + dr, c + dc
#                         if 0 <= nr < rows and 0 <= nc < cols:
#                             expanded_grid[nr][nc] = 1
#     return expanded_grid

# def find_shortest_path(grid, start, goal, a):
#     """
#     Znajduje najkrótszą ścieżkę z punktu start do goal w macierzy z uwzględnieniem szerokości obiektu.
#     """
#     # Rozszerz przeszkody
#     grid = expand_obstacles(grid, a)

#     rows, cols = len(grid), len(grid[0])
#     # Dodano ruchy na skos
#     directions = [
#         (-1, 0), (1, 0), (0, -1), (0, 1),  # Ruchy poziome i pionowe
#         (-1, -1), (-1, 1), (1, -1), (1, 1)  # Ruchy diagonalne (na skos)
#     ]

#     # Kolejka BFS
#     queue = deque([(start, [start])])
#     visited = set()
#     visited.add(start)

#     while queue:
#         (x, y), path = queue.popleft()

#         if (x, y) == goal:
#             return path

#         for dx, dy in directions:
#             nx, ny = x + dx, y + dy
#             if 0 <= nx < rows and 0 <= ny < cols and grid[nx][ny] == 0 and (nx, ny) not in visited:
#                 visited.add((nx, ny))
#                 queue.append(((nx, ny), path + [(nx, ny)]))

#     return None  # Jeśli brak ścieżki





# def heuristic(a, b):
#     """
#     Oblicza heurystykę (odległość Manhattan) między punktami a i b.
#     """
#     return abs(a[0] - b[0]) + abs(a[1] - b[1])

# def find_shortest_path_a_star(grid, start, goal, a):
#     """
#     Znajduje najkrótszą ścieżkę z punktu start do goal w macierzy z użyciem A*.
#     """
#     # Rozszerz przeszkody
#     grid = expand_obstacles(grid, a)

#     rows, cols = len(grid), len(grid[0])
#     # Dodano ruchy na skos
#     directions = [
#         (-1, 0), (1, 0), (0, -1), (0, 1),  # Ruchy poziome i pionowe
#         (-1, -1), (-1, 1), (1, -1), (1, 1)  # Ruchy diagonalne (na skos)
#     ]

#     # Kolejka priorytetowa A*
#     open_set = []
#     heapq.heappush(open_set, (0, start))

#     # Słownik śledzący koszt dojścia do węzła
#     g_score = {start: 0}
#     # Słownik śledzący poprzednie węzły
#     came_from = {}

#     while open_set:
#         # Pobierz węzeł z najmniejszym f_score
#         _, current = heapq.heappop(open_set)

#         if current == goal:
#             # Odtwórz ścieżkę
#             path = []
#             while current in came_from:
#                 path.append(current)
#                 current = came_from[current]
#             path.append(start)
#             return path[::-1]  # Odwróć ścieżkę

#         for dx, dy in directions:
#             neighbor = (current[0] + dx, current[1] + dy)

#             # Sprawdź, czy neighbor jest w granicach i dostępny
#             if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor[0]][neighbor[1]] == 0:
#                 tentative_g_score = g_score[current] + 1  # Koszt sąsiada

#                 if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
#                     # Zaktualizuj najlepszą ścieżkę do sąsiada
#                     g_score[neighbor] = tentative_g_score
#                     f_score = tentative_g_score + heuristic(neighbor, goal)
#                     heapq.heappush(open_set, (f_score, neighbor))
#                     came_from[neighbor] = current

#     return None  # Jeśli brak ścieżki






# def generate_matrix(n, zero_probability=0.9):
#     """
#     Generuje macierz n x n wypełnioną zerami i jedynkami,
#     gdzie zero_probability określa prawdopodobieństwo wstawienia zera.

#     :param n: Rozmiar macierzy (n x n)
#     :param zero_probability: Prawdopodobieństwo wstawienia zera (od 0 do 1)
#     :return: Macierz n x n
#     """
#     matrix = [
#         [0 if random.random() < zero_probability else 1 for _ in range(n)]
#         for _ in range(n)
#     ]
#     return matrix

# # Przykład użycia
# n = 25  # Rozmiar macierzy
# matrix = generate_matrix(n)


# # Przykład użycia
# grid = [
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0,],
#     [0, 0, 1, 0, 0, 0, 0, 0, 0, 0,]
# ]
# start_grid = grid.copy()
# # start_grid[0][0] = 2
# # start_grid[7][7] = 3
# #print(start_grid)
# start = (0, 0)
# goal = (n-1, n-1)
# a =0 # Szerokość obiektu
# # Wyświetlenie macierzy


# #path = find_shortest_path(matrix, start, goal, a)
# path = find_shortest_path_a_star(matrix, start, goal, a)



# if path:
#     print("Najkrótsza ścieżka:", path)
# else:
#     print("Nie znaleziono ścieżki.")

# for step in path:
#     matrix[step[0]][step[1]] = 0.5

# matrix_np = np.array(matrix)
# matrix_np = cv2.resize(matrix_np,(500,500), interpolation=cv2.INTER_NEAREST)

# cv2.imshow("path",matrix_np)

# cv2.waitKey(0)

# # Zamknij wszystkie okna
# cv2.destroyAllWindows()