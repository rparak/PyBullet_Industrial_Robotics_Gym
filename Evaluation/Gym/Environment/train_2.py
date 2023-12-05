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
import numpy as np

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.05)

#n_actions = env.action_space.shape[0]
#noise = NormalActionNoise(mean = np.zeros(n_actions), sigma = 0.1 * np.ones(n_actions))

model = DDPG(policy="MultiInputPolicy", env=env, verbose=1, gamma=0.95, learning_rate = 1e-3)
model.learn(1000)
model.save('model')


# ...
#env = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.05)

#model = DDPG(policy="MultiInputPolicy", env=env)
#model.learn(1000)
#model.save("IndustrialRoboticsReach-v0-model")