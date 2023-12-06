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

from stable_baselines3.common.logger import read_csv

import matplotlib.pyplot as plt

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

# ...
data = read_csv('./DDPG_results/progress.csv')

print(data)
#print(data['time/episodes'])

fig, ax = plt.subplots()
#x.plot(x, y_est, '-')
#ax.fill_between(data['time/episodes'], data['rollout/ep_rew_mean'], data['rollout/ep_len_mean'], alpha=0.2)

ax.plot(data['time/episodes'], data['rollout/ep_rew_mean'])

plt.show()