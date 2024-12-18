import gymnasium as gym
from tugbot_env import TugbotEnv
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import SubprocVecEnv, VecTransposeImage
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
import argparse

# Register the environment
gym.register(
    id="tugbot_env/Tugbot-v0",
    entry_point=TugbotEnv,
)

def make_env(rank):
    def _init():
        env = TugbotEnv(render_mode=None, margin=15, max_steps=1000)
        env = Monitor(env)
        return env
    return _init

def main():
    # Add argument parsing
    parser = argparse.ArgumentParser(description='Train the Tugbot RL agent')
    parser.add_argument('--resume', type=str, help='Path to the model to resume training from')
    args = parser.parse_args()

    # Initialize multiple environments in parallel
    num_envs = 24  # Adjust this based on your CPU cores
    # env = make_vec_env(TugbotEnv, n_envs=num_envs, env_kwargs={'render_mode': None, 'margin': 10, 'max_steps': 1000})
    env = SubprocVecEnv([make_env(i) for i in range(num_envs)])

    # Add VecTransposeImage if your observation space includes images
    # If your observation space is a Dict with images, uncomment this:
    # env = VecTransposeImage(env)

    # Create or load the model based on resume argument
    if args.resume:
        print(f"Resuming training from {args.resume}")
        model = PPO.load(args.resume, env=env, device='cpu')
    else:
        model = PPO(
            "MlpPolicy", 
            env, 
            verbose=1, 
            device='cpu',  # Use 'cuda' for GPU training
            learning_rate=1.0e-4,  # Reduced learning rate for stability
            n_steps=1024,        # Increased steps per update
            batch_size=32,       # Smaller batch size
            n_epochs=5,         # More epochs per update
            gamma=0.99,
            gae_lambda=0.95,     # GAE parameter
            ent_coef=0.0,      # Increased entropy for better exploration
            clip_range=0.2,      # PPO clip range
            max_grad_norm=0.5,   # Gradient clipping
            policy_kwargs=dict(
                net_arch=dict(
                    pi=[64, 64],    # Larger actor network
                    vf=[64, 64]     # Larger critic network
                ),
                ortho_init=True       # Orthogonal initialization
            )
        )
    # model = PPO.load("tugbot_final_model", env=env, device="cuda")

    # Create a separate environment for evaluation
    eval_env = SubprocVecEnv([make_env(i) for i in range(1)])

    # Setup EvalCallback to save the best model
    eval_callback = EvalCallback(
        eval_env=eval_env,
        best_model_save_path="./best_model/",
        log_path="./eval_logs/",
        eval_freq=2_000,  # Evaluate every 10,000 steps
        deterministic=True,  # Use deterministic actions during evaluation
        render=False
    )

    # Train the model
    total_timesteps = 500_000  # Start with fewer steps to test
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=eval_callback,  # Add the EvalCallback here
            progress_bar=True,
            tb_log_name="tugbot"
        )
    except KeyboardInterrupt:
        model.save("tugbot_interrupt_model")

    model.save("tugbot_final_model")

if __name__ == "__main__":
    main()
