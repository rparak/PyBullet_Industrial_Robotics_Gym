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

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str

def main():
    # ...
    gym_environment = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Sparse', distance_threshold=0.05)

    # ..
    observations, informations = gym_environment.reset()

    # ...
    for _ in range(1000):
        # ...
        action = gym_environment.action_space.sample()

        # ...
        observations, reward, successful, warning, informations = gym_environment.step(action)

        if successful == True or warning == True:
            observations, informations = gym_environment.reset()

    gym_environment.close()

if __name__ == '__main__':
    main()