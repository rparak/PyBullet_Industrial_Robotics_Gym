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

from stable_baselines3 import DDPG, HerReplayBuffer
from stable_baselines3 .common.noise import NormalActionNoise
from stable_baselines3.common.logger import configure
import numpy as np

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

tmp_path = "/tmp/DDPG_results/"
# set up logger
new_logger = configure(tmp_path, ["stdout", "csv"])

env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', 
               action_step_factor=0.04, distance_threshold=0.02)

# The noise objects for DDPG
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1*np.ones(n_actions))

model = DDPG(policy="MultiInputPolicy", env=env, replay_buffer_class=HerReplayBuffer, gamma=0.95, learning_rate=0.001, action_noise=action_noise, verbose=1)
# Set new logger
model.set_logger(new_logger)
model.learn(total_timesteps=10000, log_interval=10)
model.save('model')

# ...
#env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.05)

#model = DDPG(policy="MultiInputPolicy", env=env)
#model.learn(1000)
#model.save("IndustrialRoboticsReach-v0-model")