import pygame
import numpy as np
import gymnasium as gym
from typing import Union
from gymnasium import spaces
from tugbot_pygame import MAX_SPEED, MAX_OMEGA, WIDTH, HEIGHT, BLACK
from tugbot_pygame import reset_environment, update_position, distance, generate_path, draw_path, draw_robot, display_info

class TugbotEnv(gym.Env):
    def __init__(self, render_mode: str='human', margin: int=20, point_dist: int = 500, max_steps: Union[int, None]=None):
        super(TugbotEnv, self).__init__()
        self.render_mode = render_mode
        self.margin = margin
        self.point_dist = point_dist
        self.max_steps = max_steps
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array([-1, -1, -1, -1, -1, -1, -1, -1, -1]),  # Added 2 more dimensions for previous action
            high=np.array([1, 1, 1, 1, 1, 1, 1, 1, 1]),
            dtype=np.float32
        )
        self.action = np.zeros(2, dtype=np.float32)
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.screen = None
        self.max_distance = 300  # Maximum allowed distance from target point
        self.last_distance = None
        self.last_angle_diff = None
        self.min_speed = 20  # Minimum speed threshold
        self.reset()
        self.init_pygame()
        self.last_progress = 0
        self.movement_history = []
        self.oscillation_window = 10

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
        self.steps = 0  # Initialize step counter
        self.prev_x, self.prev_y = self.x, self.y  # Store initial position
        self.prev_action = np.zeros(2, dtype=np.float32)
        
        # Initialize distances and angles
        self.last_distance = distance((self.x, self.y), self.path[self.next_point_index])
        self.last_angle_diff = None
        
        state = np.array(self.get_state(), dtype=np.float32)
        return state, {}

    def calculate_reward(self):
        dist_to_next_point = distance((self.x, self.y), self.path[self.next_point_index])
        
        # Progress reward with momentum
        progress = self.last_distance - dist_to_next_point
        progress_reward = progress * 5.0  # Increased weight for progress
        
        # Track progress history for oscillation detection
        self.movement_history.append(progress)
        if len(self.movement_history) > self.oscillation_window:
            self.movement_history.pop(0)
        
        # Penalize oscillations
        oscillation_penalty = 0
        if len(self.movement_history) == self.oscillation_window:
            changes = sum(1 for i in range(1, len(self.movement_history)) 
                         if (self.movement_history[i] * self.movement_history[i-1]) < 0)
            if changes > self.oscillation_window / 2:
                oscillation_penalty = -2.0

        self.last_distance = dist_to_next_point

        # Improved angle alignment reward
        desired_theta = np.arctan2(
            self.path[self.next_point_index][1] - self.y,
            self.path[self.next_point_index][0] - self.x,
        )
        angle_diff = abs(desired_theta - self.theta) % (2 * np.pi)
        if angle_diff > np.pi:
            angle_diff = 2.0 * np.pi - angle_diff
        
        heading_reward = np.cos(angle_diff)  # Smoother angle reward

        # Dynamic speed reward
        speed_factor = abs(self.v) / MAX_SPEED
        angle_factor = (np.pi - angle_diff) / np.pi
        speed_reward = speed_factor * angle_factor * 2.0  # Reward higher speeds when aligned
        
        if abs(self.v) < self.min_speed and angle_diff < np.pi/4:
            speed_reward -= 1.0  # Stronger penalty for stopping when aligned

        # Safety penalties
        safety_penalty = 0
        if abs(self.omega) > MAX_OMEGA * 0.8:  # Excessive rotation
            safety_penalty -= 2.0
        if abs(self.v) > MAX_SPEED * 0.9:  # Near maximum speed
            safety_penalty -= 1.0

        # Checkpoint reward
        checkpoint_reward = 0
        if dist_to_next_point < self.margin:
            checkpoint_reward = 100.0
            if self.next_point_index == len(self.path) - 1:
                self.path = generate_path(self.x, self.y, width=WIDTH, height=HEIGHT, step_size=self.point_dist)
                self.next_point_index = 1
                checkpoint_reward += 200.0
            else:
                self.next_point_index += 1

        return (progress_reward + 
                dist_to_next_point * -0.2 +
                heading_reward * 0.5 + 
                speed_reward * 1.5+ 
                safety_penalty * 1.5 +
                oscillation_penalty * 0.5 + 
                checkpoint_reward)

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

        state = np.array(self.get_state(), dtype=np.float32)
        return state, reward, done, truncated, {}

    def get_state(self):
        next_point = self.path[self.next_point_index]
        next_next_point = self.path[self.next_point_index + 1] if self.next_point_index + 1 < len(self.path) else next_point
        dx1 = (next_point[0] - self.x) / WIDTH
        dy1 = (next_point[1] - self.y) / HEIGHT
        dx2 = (next_next_point[0] - self.x) / WIDTH
        dy2 = (next_next_point[1] - self.y) / HEIGHT
        return [
            self.v / MAX_SPEED, 
            self.omega / MAX_OMEGA, 
            dx1, dy1, dx2, dy2, 
            self.theta / np.pi,
            self.prev_action[0],  # Add previous actions to state
            self.prev_action[1]
        ]

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
