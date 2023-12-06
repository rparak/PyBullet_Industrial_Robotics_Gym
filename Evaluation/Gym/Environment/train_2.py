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
from stable_baselines3 .common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import numpy as np

import time
#import torch
#torch.cuda.set_device(1)


# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

tmp_path = "./DDPG_results"
# set up logger
new_logger = configure(tmp_path, ['stdout', 'csv'])

# ...
env = Monitor(gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', 
                        action_step_factor=0.04, distance_threshold=0.02))
env = DummyVecEnv([lambda: env])

# The noise objects for DDPG
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1*np.ones(n_actions))

t_0 = time.time()
model = DDPG(policy="MultiInputPolicy", env=env, gamma=0.95, learning_rate=0.001, action_noise=action_noise, device='cuda', verbose=1)
# Set new logger
model.set_logger(new_logger)
model.learn(total_timesteps=100000, log_interval=10)
model.save('model')

print(time.time() - t_0)

# ...
#env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.05)

#model = DDPG(policy="MultiInputPolicy", env=env)
#model.learn(1000)
#model.save("IndustrialRoboticsReach-v0-model")
