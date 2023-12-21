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

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# The name of the environment mode.
#   'Default': 
#       The mode called "Default" demonstrates an environment without a collision object.
#   'Collision-Free': 
#       The mode called "Collision-Free" demonstrates an environment with a collision object.
CONST_ENV_MODE = 'Collision-Free'

def main():
    """
    Description:
        A program for a simple test of a pre-defined environment.
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Create the environment that was previously registered using gymnasium.register() within the __init__.py file.
    #   More information can be found in the following script:
    #       ../src/Industrial_Robotics_Gym/__init__.py
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_ENV_MODE), T=None)

    # Reset the pre-defined environment of the gym.
    #   Note:
    #       Obtain the initial information and observations.
    observations, informations = gym_environment.reset()

    for _ in range(1000):
        # Obtain a random action sample from the entire action space.
        action = gym_environment.action_space.sample()

        # Perform the action within the pre-defined environment and get the new observation space.
        observations, reward, terminated, truncated, informations = gym_environment.step(action)

        # When the reach task process is terminated or truncated, reset the pre-defined gym environment.
        if terminated == True or truncated == True:
            observations, informations = gym_environment.reset()

    # Disconnect the created environment from a physical server.
    gym_environment.close()

if __name__ == '__main__':
    main()