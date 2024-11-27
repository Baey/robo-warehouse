import pygame
import numpy as np
import gymnasium as gym
from typing import Union
from gymnasium import spaces
from tugbot_pygame import MAX_SPEED, MAX_OMEGA, WIDTH, HEIGHT, BLACK
from tugbot_pygame import reset_environment, update_position, distance, generate_path, draw_path, draw_robot, display_info

class TugbotEnv(gym.Env):
    def __init__(self, render_mode: str='human', margin: int=20, point_dist: int = 40, max_steps: Union[int, None]=None):
        super(TugbotEnv, self).__init__()
        self.render_mode = render_mode
        self.margin = margin
        self.point_dist = point_dist
        self.max_steps = max_steps
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "rot": spaces.Box(-2*np.pi, 2*np.pi, shape=(1,), dtype=np.float32),
                "lin_vel": spaces.Box(-MAX_SPEED, MAX_SPEED, shape=(1,), dtype=np.float32),
                "ang_vel": spaces.Box(-MAX_OMEGA, MAX_OMEGA, shape=(1,), dtype=np.float32),
                "dist": spaces.Box(-WIDTH, WIDTH, shape=(2,), dtype=np.float32)
            }
        )
        self.action = np.zeros(2, dtype=np.float32)
        # self.prev_action = np.zeros(2, dtype=np.float32)
        self.screen = None
        self.max_distance = 600  # Maximum allowed distance from target point
        self.last_distance = None
        self.progress_reward = 0.0
        # self.last_angle_diff = None
        self.min_speed = 20  # Minimum speed threshold
        self.reset()
        self.init_pygame()
        self.last_progress = 0

    def init_pygame(self):
        if self.render_mode == 'human' and self.screen is None:
            pygame.init()
            self.screen = pygame.display.set_mode((WIDTH, HEIGHT))

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.x, self.y, self.theta, self.path = reset_environment(WIDTH, HEIGHT)
        self.v = 0.
        self.omega = 0.
        self.potential = self.point_dist
        self.next_point_index = 1
        self.steps = 0
        # self.prev_x, self.prev_y = self.x, self.y  # Store initial position
        # self.prev_action = np.zeros(2, dtype=np.float32)
        
        # Initialize distances and angles
        self.last_distance = distance((self.x, self.y), self.path[self.next_point_index])
        self.last_angle_diff = None
        
        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def calculate_reward(self):
        dist_to_next_point = distance((self.x, self.y), self.path[self.next_point_index])
        
        # Progress reward with momentum
        progress = self.last_distance - dist_to_next_point
        progress_reward = progress

        self.last_distance = dist_to_next_point

        # Improved angle alignment reward
        desired_theta = np.arctan2(
            self.path[self.next_point_index][1] - self.y,
            self.path[self.next_point_index][0] - self.x,
        )
        angle_diff = abs(desired_theta - self.theta) % (2 * np.pi)
        if angle_diff > np.pi:
            angle_diff = 2.0 * np.pi - angle_diff
        
        heading_reward = np.cos(angle_diff)

        # Checkpoint reward
        if dist_to_next_point < self.margin:
            self.progress_reward += 100.0
            if self.next_point_index == len(self.path) - 1:
                self.path = generate_path(self.x, self.y, width=WIDTH, height=HEIGHT, step_size=self.point_dist, num_points=16)
                self.next_point_index = 1
            else:
                self.next_point_index += 1

        return (self.progress_reward * 1.0 + 
                dist_to_next_point * -0.2 +
                heading_reward * 0.5)

    def step(self, action):
        self.prev_action = self.action.copy()
        self.action = action
        target_v = action[0] * MAX_SPEED
        target_omega = action[1] * MAX_OMEGA

        # Apply inertia
        self.v = self.v * 0.95 + target_v * 0.05
        self.omega = self.omega * 0.9 + target_omega * 0.1

        dt = 1 / 60.0  # Assuming a fixed time step for simplicity
        self.prev_x, self.prev_y = self.x, self.y  # Store previous position
        self.x, self.y, self.theta = update_position(self.x, self.y, self.theta, self.v, self.omega, dt)

        reward = self.calculate_reward()
        done = False
        truncated = False

        # Check if agent is too far from target point
        dist_to_next_point = distance((self.x, self.y), self.path[self.next_point_index])
        if dist_to_next_point > self.max_distance:
            done = True
            reward -= 500  # Penalty for straying too far

        # Increment step counter and check for timeout
        self.steps += 1
        if self.max_steps is not None and self.steps >= self.max_steps:
            done = True
            truncated = True

        obs = self._get_obs()
        return obs, reward, done, truncated, self._get_info()

    def _get_obs(self):
        next_point = self.path[self.next_point_index]
        dx = (next_point[0] - self.x)
        dy = (next_point[1] - self.y)
        obs = {
            "rot": self.theta,
            "lin_vel": self.v,
            "ang_vel": self.omega,
            "dist": np.array([dx, dy], dtype=np.float32)
        }

        return obs
    
    def _get_info(self):
        return {
            "x": self.x,
            "y": self.y,
            "theta": self.theta,
            "v": self.v,
            "omega": self.omega,
            "action": self.action,
            "next_point": self.path[self.next_point_index]
        }

    def render(self):
        if self.render_mode == 'human':
            self.screen.fill((255, 255, 255))
            draw_path(self.screen, self.path, self.next_point_index)
            draw_robot(self.screen, self.x, self.y, self.theta)
            display_info(self.screen, self.x, self.y, self.path[self.next_point_index], distance((self.x, self.y), self.path[self.next_point_index]))
            self.display_action_info()
            pygame.display.flip()

    def display_action_info(self):
        font = pygame.font.Font(None, 36)
        text = font.render(f"Action: {self.action[0]:.2f} {self.action[1]:.2f}", True, BLACK)
        self.screen.blit(text, (20, 60))

    def close(self):
        pygame.quit()
        self.screen = None
