import gymnasium as gym
import torch
import pygame
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from tugbot_env import TugbotEnv

# Register the environment
gym.register(
    id="tugbot_env/Tugbot-v0",
    entry_point=TugbotEnv,
)

def main():
    device = torch.device("cpu")
    
    # Create and wrap the environment with explicit rendering
    env = TugbotEnv(render_mode='human', margin=20, max_steps=1000)
    base_env = env  # Store reference to unwrapped env
    env = Monitor(env)
    env = DummyVecEnv([lambda: env])

    try:
        # Load the trained model
        # model = PPO.load("/home/developer/ros2_ws/src/path_following_rl/best_model/best_model.zip", env=env, device=device)
        # model = PPO.load("/home/developer/ros2_ws/src/path_following_rl/tugbot_final_model.zip", env=env, device=device)
        # model = PPO.load("/home/developer/ros2_ws/src/path_following_rl/best_model/good_model.zip", env=env)
        model = PPO.load("/home/developer/ros2_ws/src/path_following_rl/tugbot_interrupt_model.zip", device='cpu')
        
        # Run episodes
        num_episodes = 5
        for episode in range(num_episodes):
            obs = env.reset()  # VecEnv doesn't return info
            episode_reward = 0
            done = False
            
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, info = env.step(action)  # VecEnv returns different format
                episode_reward += reward[0]  # reward is now a numpy array
                done = done[0]  # done is now a numpy array
                
                # Ensure the window updates
                base_env.render()
                pygame.display.flip()
                pygame.time.wait(2)  # Add small delay to make visualization smooth
                
            print(f"Episode {episode + 1} reward: {episode_reward}")
            
    except Exception as e:
        print(f"Error loading or running model: {e}")
    finally:
        env.close()
        pygame.quit()

if __name__ == "__main__":
    main()