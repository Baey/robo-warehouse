import torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from stable_baselines3.common.callbacks import EvalCallback
from tugbot_env import TugbotEnv

# Create environments
def make_env(render_mode=None):
    def _init():
        env = TugbotEnv(render_mode=render_mode, margin=20, max_steps=8000)
        return Monitor(env)
    return _init

# Create training environment
train_envs = [make_env() for _ in range(8)]
vec_env = DummyVecEnv(train_envs)
vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True)

# Create evaluation environment (wrapped the same way as training)
eval_envs = [make_env()]
eval_env = DummyVecEnv(eval_envs)
eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=True, training=False)

# Create evaluation callback
eval_callback = EvalCallback(
    eval_env,
    best_model_save_path='./best_model/',
    log_path='./logs/',
    eval_freq=10000,
    deterministic=True,
    render=False
)

# Configure PPO with better hyperparameters
model = PPO(
    "MlpPolicy",
    vec_env,
    learning_rate=3e-4,  # More conservative learning rate
    n_steps=2048,
    batch_size=64,  # Smaller batch size for better generalization
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_range=0.2,
    ent_coef=0.01,  # Reduced entropy for more exploitation
    policy_kwargs=dict(
        net_arch=dict(
            pi=[128, 128],  # Separate policy network
            vf=[128, 128]   # Separate value network
        )
    ),
    verbose=1,
    device='cpu'
)

# Increase training time for better learning
model.learn(total_timesteps=5_000_000, callback=eval_callback)

# Save the final model
model.save("ppo_tugbot_final")
vec_env.save("vec_normalize.pkl")

# Test visualization
env = TugbotEnv(render_mode='human', margin=20)
obs, _ = env.reset()
done = False

while not done:
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    env.render()