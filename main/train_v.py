from race_env import RacingEnv
from stable_baselines3 import PPO

model = PPO.load("ppo_racing")

env = RacingEnv(gui=True)
obs, _ = env.reset()

while True:
    action, _ = model.predict(obs, deterministic=True)
    obs, _, done, _, _ = env.step(action)
    if done:
        obs, _ = env.reset()
