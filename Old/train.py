# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# OS (Operating system interfaces)
import os
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   Industrial_Robotics_Gym
#       ../Industrial_Robotics_Gym
import Industrial_Robotics_Gym

from stable_baselines3 import DDPG
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv

# stable_baselines3 VecMonitor

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
        
def main():
    # Save a checkpoint every 1000 steps
    checkpoint_callback = CheckpointCallback(
        save_freq=1000,
        save_path="./logs/",
        name_prefix="rl_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
    )

    # ...
    env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.05)

    # ..
    env = DummyVecEnv([lambda: env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True)

    # ...
    model = DDPG(policy="MultiInputPolicy", env=env, verbose=1)
    model.learn(total_timesteps=1000, callback=checkpoint_callback)
    model.save("IndustrialRoboticsReach-v0-model")

if __name__ == '__main__':
    main()