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
    gym_environment = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', distance_threshold=0.04)

    # ..
    observations, informations = gym_environment.reset()

    # ...
    for _ in range(1000):
        # ...
        action = gym_environment.action_space.sample()

        # ...
        observations, reward, terminated, truncated, informations = gym_environment.step(action)

        if terminated == True or truncated == True:
            if terminated == True:
                print('Done!')
                break
            observations, informations = gym_environment.reset()

    gym_environment.close()

if __name__ == '__main__':
    main()