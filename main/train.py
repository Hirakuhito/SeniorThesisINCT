from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from race_env import RacingEnv

def make_env():
    return RacingEnv(gui=False)

env = DummyVecEnv([make_env])

model = PPO(
    "MlpPolicy",
    env,
    device="cuda",


    n_steps=2048,
    batch_size=64,
    n_epochs=100,

    gamma=0.99,
    gae_lambda=0.95,

    learning_rate=3e-4,
    clip_range=0.2,

    ent_coef=0.0,
    vf_coef=0.5,
    max_grad_norm=0.5,

    verbose=1,
)

model.learn(total_timesteps=20_000)
model.save("ppr_racing")