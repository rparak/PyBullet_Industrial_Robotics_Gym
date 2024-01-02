# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# OS (Operating system interfaces)
import os
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
#   PyBullet
#       ../PyBullet/Utilities
import PyBullet.Utilities

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
CONST_ALGORITHM = 'TD3_HER'
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program designed for the prediction of a 'reach' task in a pre-defined environment, utilizing 
        the Twin Delayed DDPG (TD3) reinforcement learning algorithm.

        The DDPG algorithm is pre-trained only for the following robotic arms:
            - Universal Robots UR3
            
        Note:
            The goal will be statically defined as the center of the target configuration space.

        More information about the training process can be found in the script below:
            ../PyBullet_Industrial_Robotics_Gym/Training/train_{CONST_ALGORITHM}.py
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The specified path of the file to save the data.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/path_static_target'
    
    # Removes old files (if any) created by the previous prediction.
    if os.path.isfile(f'{file_path}.txt'):
        os.remove(f'{file_path}.txt')

    # Obtain the structure of the main parameters of the environment for the defined robotic arm.
    Env_Structure = PyBullet.Utilities.Get_Environment_Structure(Robot_Str.Name, 0 if CONST_ENV_MODE == 'Default' else 1)

    # Create the environment that was previously registered using gymnasium.register() within the __init__.py file.
    #   More information can be found in the following script:
    #       ../src/Industrial_Robotics_Gym/__init__.py
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_ENV_MODE), T=Env_Structure.C.Target.T)

    # Create a vectorized environment.
    gym_environment = stable_baselines3.common.vec_env.DummyVecEnv([lambda: gym_environment])

    # Load a pre-trained model from a zip file.
    model = stable_baselines3.TD3.load(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model')
    
    # Reset the pre-defined environment of the gym.
    #   Note:
    #       Obtain the initial information and observations.
    observations, informations = gym_environment.reset()

    while True:
        # Get the policy action from an observation.
        action, _ = model.predict(observations)

        # Perform the action within the pre-defined environment and get the new observation space.
        observations, reward, terminated, truncated, informations = gym_environment.step(action)

        # Save the data to the '*.txt' file.
        RoLE.Utilities.File_IO.Save(file_path, observations['achieved_goal'], 'txt', ',')
        
        # When the reach task process is terminated or truncated, reset the pre-defined gym environment.
        if terminated == True or truncated == True:
            observations, informations = gym_environment.reset()
            break

    # Disconnect the created environment from a physical server.
    gym_environment.close()

if __name__ == '__main__':
    sys.exit(main())