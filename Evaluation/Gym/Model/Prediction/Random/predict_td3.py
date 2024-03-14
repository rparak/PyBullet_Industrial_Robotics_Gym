# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../../' + 'src')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Stable-Baselines3 (A set of implementations of reinforcement learning algorithms in PyTorch) [pip3 install stable-baselines3]
import stable_baselines3 
import stable_baselines3.common.vec_env
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
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
CONST_ENV_MODE = 'Default'
# The name of the reinforcement learning algorithm. 
#   'TD3':
#       Twin Delayed DDPG (TD3) Addressing Function Approximation Error in Actor-Critic Methods. TD3 is a direct successor of DDPG and improves 
#       it using three major tricks: clipped double Q-Learning, delayed policy update and target policy smoothing.
#   'TD3_HER':
#       Twin Delayed DDPG (TD3) + Hindsight Experience Replay (HER)
CONST_ALGORITHM = 'TD3'
# Number of randomly generated targets.
CONST_N_TARGETS = 100
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program designed for the prediction of a 'reach' task in a pre-defined environment, utilizing 
        the Twin Delayed DDPG (TD3) reinforcement learning algorithm.

        The DDPG algorithm is pre-trained only for the following robotic arms:
            - Universal Robots UR3 -> Both environments and each algorithm.
            - Default Env. -> Only the TD3 algorithm for all robotic structures.

        Note:
            Targets will be defined randomly.

        More information about the training process can be found in the script below:
            ../PyBullet_Industrial_Robotics_Gym/Training/train_{CONST_ALGORITHM}.py
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The specified path of the file to save the data.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/random_N_{CONST_N_TARGETS}'
    
    # Removes old files (if any) created by the previous prediction.
    if os.path.isfile(f'{file_path}.txt'):
        os.remove(f'{file_path}.txt')

    # Create the environment that was previously registered using gymnasium.register() within the __init__.py file.
    #   More information can be found in the following script:
    #       ../src/Industrial_Robotics_Gym/__init__.py
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_ENV_MODE), T=None)

    # Load a pre-trained model from a zip file.
    if 'HER' in CONST_ALGORITHM:
        model = stable_baselines3.TD3.load(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model', env=gym_environment)
    else:
        model = stable_baselines3.TD3.load(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model')
    
    # Reset the pre-defined environment of the gym.
    #   Note:
    #       Obtain the initial information and observations.
    observations, informations = gym_environment.reset()

    i = 0; episode_length = 1; reward_sum = 0.0
    while i < CONST_N_TARGETS:
        # Get the policy action from an observation.
        action, _ = model.predict(observations)

        # Perform the action within the pre-defined environment and get the new observation space.
        observations, reward, terminated, truncated, informations = gym_environment.step(action)
        
        # Get a current reward.
        reward_sum += reward

        # When the reach task process is terminated or truncated, reset the pre-defined gym environment.
        if terminated == True or truncated == True:
            # Get the Euclidean distance.
            d = np.linalg.norm(observations['achieved_goal'] - observations['desired_goal'], axis=-1)

            # Save the data to the '*.txt' file.
            RoLE.Utilities.File_IO.Save(file_path, np.array([i + 1, informations['is_success'], reward_sum, episode_length, 
                                                             d]), 'txt', ',')

            # Reset the pre-defined environment of the gym.
            observations, informations = gym_environment.reset()
            episode_length = 0; i += 1; reward_sum = 0.0

            print(f'[INFO] Progress: {i} / {CONST_N_TARGETS}')

        episode_length += 1

    # Disconnect the created environment from a physical server.
    gym_environment.close()

if __name__ == '__main__':
    sys.exit(main())