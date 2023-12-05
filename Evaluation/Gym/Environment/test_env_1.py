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
    gym_environment = gym.make('IndustrialRoboticsReach-v0', mode='Default', Robot_Str=CONST_ROBOT_TYPE, reward_type='Dense', 
                               action_step_factor=0.05, distance_threshold=0.01)

    # ..
    observations, informations = gym_environment.reset()

    terminated = False; i = 0
    while not terminated:
        current_position = observations['achieved_goal']
        desired_position = observations['desired_goal']
        action = desired_position - current_position
        observations, reward, terminated, truncated, informations = gym_environment.step(action)
        i += 1

    print(i)
    gym_environment.close()

if __name__ == '__main__':
    main()