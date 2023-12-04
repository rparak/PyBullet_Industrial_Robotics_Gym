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

    successful = False
    while not successful:
        current_position = observations['p'][0:3]
        desired_position = observations['p_1'][0:3]
        action = 5.0 * (desired_position - current_position)
        observations, reward, successful, warning, informations = gym_environment.step(action)

    gym_environment.close()

if __name__ == '__main__':
    main()