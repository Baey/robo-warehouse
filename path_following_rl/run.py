
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from tugbot_env import TugbotEnv

# Check if MPS is available and set the device
# device = torch.device("mps") if torch.backends.mps.is_available() else torch.device("cpu")
device = torch.device("cpu")

env = Monitor(TugbotEnv(render_mode='human', margin=20, max_steps=8000))

# Load the model
model = PPO.load("ppo_tugbot_final", env=env, device=device)

# Test model
obs, _ = env.reset()  # Unpack the tuple returned by reset
done = False
while not done:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)  # Unpack the tuple returned by step
    env.render()  # Ensure render is called within the loop to update the window