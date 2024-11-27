import gymnasium as gym
from tugbot_env import TugbotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback

# Register the environment
gym.register(
    id="tugbot_env/Tugbot-v0",
    entry_point=TugbotEnv,
)

def main():
    # Initialize the environment in headless mode
    env = TugbotEnv(render_mode=None, margin=20, max_steps=5000)
    env = Monitor(env)
    env = DummyVecEnv([lambda: env])

    # Create the model
    model = PPO(
        "MultiInputPolicy", 
        env, 
        verbose=1, 
        device='cpu',
        learning_rate=1e-4,      # Reduced learning rate
        n_steps=1024,            # Reduced steps per update
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01           # Added entropy coefficient for exploration
    )

    # Setup checkpointing
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path="./training_checkpoints/",
        name_prefix="tugbot_model"
    )

    # Train the model
    total_timesteps = 1_000_000
    model.learn(
        total_timesteps=total_timesteps,
        callback=checkpoint_callback,
        progress_bar=True
    )

    # Save the final model
    model.save("tugbot_final_model")

if __name__ == "__main__":
    main()