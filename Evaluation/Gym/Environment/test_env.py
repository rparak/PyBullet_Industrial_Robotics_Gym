# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   Industrial_Robotics_Gym
#       ../Industrial_Robotics_Gym
import Industrial_Robotics_Gym
#       ../Industrial_Robotics_Gym/Utilities
import Industrial_Robotics_Gym.Utilities

from stable_baselines3.common.env_checker import check_env

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# ...
CONST_MODE = 'Default'

def main():

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # ...
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_MODE))

    check_env(gym_environment, warn=True)

    """
    # ..
    observations, informations = gym_environment.reset()

    # ...
    action = gym_environment.action_space.sample()

    # ...
    print(action, type(action))
    # ...
    observations, reward, terminated, truncated, informations = gym_environment.step(action)
    print(observations, type(observations))
    print(reward, type(reward))
    print(terminated, type(terminated))
    print(truncated, type(truncated))
    print(informations, type(informations))
    
    """

    """
    # ...
    observations, informations = gym_environment.reset()
    for _ in range(1000):
        # ...
        action = gym_environment.action_space.sample()

        # ...
        observations, reward, terminated, truncated, informations = gym_environment.step(action)

        if terminated == True or truncated == True:
            observations, informations = gym_environment.reset()

    gym_environment.close()
    """

if __name__ == '__main__':
    main()