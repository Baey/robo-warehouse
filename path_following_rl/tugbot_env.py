import pygame
import numpy as np
import gymnasium as gym
from typing import Union
from gymnasium import spaces
from tugbot_pygame import MAX_SPEED, MAX_OMEGA, WIDTH, HEIGHT, BLACK
from tugbot_pygame import reset_environment, update_position, distance, generate_path, draw_path, draw_robot, display_info
import random

class TugbotEnv(gym.Env):
    def __init__(self, render_mode: str='human', margin: int=20, point_dist: int = 50, max_steps: Union[int, None]=None):
        super(TugbotEnv, self).__init__()
        self.render_mode = render_mode
        self.margin = margin
        self.point_dist = point_dist
        self.max_steps = max_steps
        self.reward_steps = 0
        self.num_point_observations = 3  # Number of future points to observe
        self.action_space = spaces.Box(low=np.array([-1, -1]), high=np.array([1, 1]), shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -1.0] + [0.0, -1.0] * self.num_point_observations),
            high=np.array([1.0, 1.0] + [1.0, 1.0] * self.num_point_observations),
            shape=(2 + 2 * self.num_point_observations,),  # [linear_vel, angular_vel, (distance, rotation_diff) * num_points]
            dtype=np.float32
        )
        self.action = np.zeros(2, dtype=np.float32)
        self.screen = None
        self.max_distance = 450
        self.distance = None
        self.last_distance = None
        self.progress_reward = 0.0
        self.reset()
        if self.render_mode == 'human':
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
        
        self.last_distance = distance((self.x, self.y), self.path[self.next_point_index])
        self.distance = self.last_distance
        self.last_angle_diff = None
        
        self.linear_inertia = random.uniform(0.95, 0.99)  # Randomize linear inertia
        self.angular_inertia = random.uniform(0.90, 0.97)  # Randomize angular inertia
        
        obs = self._get_obs()
        info = self._get_info()
        return obs, info

    def calculate_reward(self):        
        potential_reward = self._potential_reward()
        heading_reward = self._heading_reward()
        checkpoint_reward = self._checkpoint_reward()
        speed_penalty = self._speed_penalty()
        belt_reward = self._belt_reward()  # Add belt reward

        total_reward = (
            potential_reward * 3.0 +
            checkpoint_reward * 2.0 + 
            speed_penalty * 0.01 +
            heading_reward * 0.2 +
            belt_reward * 0.4 +  # Include belt reward
            - 0.1
        )

        self.reward_components = {
            "potential_reward": potential_reward,
            "heading_reward": heading_reward,
            "checkpoint_reward": checkpoint_reward,
            "speed_penalty": speed_penalty,
            "belt_reward": belt_reward,  # Add belt reward
            "total_reward": total_reward
        }

        return total_reward

    def _potential_reward(self):
        potential_reward = -0.01
        if self.distance < self.potential:
            potential_reward = self.potential - self.distance
            self.potential = self.distance

        return potential_reward

    def _heading_reward(self):
        desired_theta = np.arctan2(
            self.path[self.next_point_index][1] - self.y,
            self.path[self.next_point_index][0] - self.x,
        )
        angle_diff = abs(desired_theta - self.theta) % (2 * np.pi)
        if angle_diff > np.pi:
            angle_diff = 2.0 * np.pi - angle_diff
        
        return np.cos(angle_diff)

    def _checkpoint_reward(self):
        checkpoint_reward = 0.0
        if self.distance < self.margin:
            self.progress_reward += 100.0
            self.reward_steps += 100
            if self.next_point_index == len(self.path) - 1:
                self.path = generate_path(self.x, self.y, width=WIDTH, height=HEIGHT, step_size=self.point_dist, num_points=16)
                self.next_point_index = 1
                self.potential = self.point_dist
            else:
                self.next_point_index += 1
                self.potential = self.distance
            self.distance = distance((self.x, self.y), self.path[self.next_point_index])
            self.last_distance = self.distance
            checkpoint_reward = self.progress_reward
        return checkpoint_reward

    def _speed_penalty(self):
        return - np.abs(self.v) / MAX_SPEED

    def _belt_reward(self):
        prev_point = self.path[self.next_point_index - 1]
        next_point = self.path[self.next_point_index]
        
        # Calculate the vector from previous to next point
        path_vector = np.array(next_point) - np.array(prev_point)
        path_length = np.linalg.norm(path_vector)
        path_unit_vector = path_vector / path_length
        
        # Calculate the vector from previous point to the robot's position
        robot_vector = np.array([self.x, self.y]) - np.array(prev_point)
        
        # Project the robot's position onto the path vector
        projection_length = np.dot(robot_vector, path_unit_vector)
        projection_point = np.array(prev_point) + projection_length * path_unit_vector
        
        # Calculate the distance from the robot to the projection point
        distance_to_path = np.linalg.norm(np.array([self.x, self.y]) - projection_point)
        
        # Define the belt width
        belt_width = 20.0
        
        # Calculate the reward based on the distance to the center of the belt
        if distance_to_path < belt_width:
            belt_reward = 1.0 - (distance_to_path / belt_width)
        else:
            belt_reward = 0.0
        
        return belt_reward

    def step(self, action):
        self.last_distance = self.distance
        self.distance = distance((self.x, self.y), self.path[self.next_point_index])
        self.prev_action = self.action.copy()
        self.action = action
        target_v = action[0] * MAX_SPEED
        target_omega = action[1] * MAX_OMEGA

        # Apply increased inertia
        self.v = self.v * self.linear_inertia + target_v * (1 - self.linear_inertia)       # More resistance to linear velocity changes
        self.omega = self.omega * self.angular_inertia + target_omega * (1 - self.angular_inertia)  # More resistance to angular velocity changes

        dt = 1 / 60.0  # Assuming a fixed time step for simplicity
        self.prev_x, self.prev_y = self.x, self.y  # Store previous position
        self.x, self.y, self.theta = update_position(self.x, self.y, self.theta, self.v, self.omega, dt)

        reward = self.calculate_reward()
        done = False
        truncated = False

        # Check if agent is too far from target point
        if self.distance > self.max_distance:
            done = True
            reward -= 50  # Penalty for straying too far

        # Increment step counter and check for timeout
        self.steps += 1
        if self.max_steps is not None and self.steps >= self.max_steps + self.reward_steps:
            done = True
            truncated = True

        obs = self._get_obs()
        return obs, reward, done, truncated, self._get_info()

    def _calculate_point_observation(self, point_index):
        """Calculate normalized distance and rotation difference for a given path point."""
        if point_index >= len(self.path):
            point_index = len(self.path) - 1
            
        point = self.path[point_index]
        
        # Calculate distance
        dist = distance((self.x, self.y), point)
        normalized_distance = dist / self.max_distance
        
        # Calculate rotation difference
        desired_theta = np.arctan2(point[1] - self.y, point[0] - self.x)
        rotation_diff = (desired_theta - self.theta) % (2 * np.pi)
        if rotation_diff > np.pi:
            rotation_diff = rotation_diff - 2 * np.pi
        normalized_rotation = rotation_diff / np.pi
        
        return normalized_distance, normalized_rotation

    def _get_obs(self):
        # Normalize velocities
        normalized_lin_vel = self.v / MAX_SPEED
        normalized_ang_vel = self.omega / MAX_OMEGA
        
        # Get observations for future points
        point_observations = []
        for i in range(self.num_point_observations):
            point_index = self.next_point_index + i
            dist, rot = self._calculate_point_observation(point_index)
            point_observations.extend([dist, rot])
        
        return np.array([normalized_lin_vel, normalized_ang_vel] + point_observations, dtype=np.float32)
    
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
            display_info(self.screen, self.x, self.y, self.path[self.next_point_index], self.distance)
            self.display_action_info()
            self.display_reward_info()
            self.display_observation_info()  # Add this line
            pygame.display.flip()

    def display_action_info(self):
        font = pygame.font.Font(None, 36)
        text = font.render(f"Action: {self.action[0]:.2f} {self.action[1]:.2f}", True, BLACK)
        self.screen.blit(text, (20, 60))

    def display_reward_info(self):
        font = pygame.font.Font(None, 24)
        y_offset = 100
        for key, value in self.reward_components.items():
            text = font.render(f"{key}: {value:.2f}", True, BLACK)
            self.screen.blit(text, (20, y_offset))
            y_offset += 30

    def display_observation_info(self):
        font = pygame.font.Font(None, 24)
        obs = self._get_obs()
        y_offset = 220
        
        text = font.render("Observations:", True, BLACK)
        self.screen.blit(text, (20, y_offset))
        y_offset += 30
        
        # Display velocities
        labels = ['Linear Velocity', 'Angular Velocity']
        for label, value in zip(labels, obs[:2]):
            text = font.render(f"{label}: {value:.2f}", True, BLACK)
            self.screen.blit(text, (20, y_offset))
            y_offset += 25
        
        # Display point observations
        for i in range(self.num_point_observations):
            base_idx = 2 + i * 2
            text = font.render(f"Point {i+1} - Distance: {obs[base_idx]:.2f}, Rotation: {obs[base_idx+1]:.2f}", True, BLACK)
            self.screen.blit(text, (20, y_offset))
            y_offset += 25

    def close(self):
        pygame.quit()
        self.screen = None
