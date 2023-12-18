# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# OS (Operating system interfaces)
import os
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Stable-Baselines3 (A set of implementations of reinforcement learning algorithms in PyTorch) [pip3 install stable-baselines3]
import stable_baselines3 
import stable_baselines3.common.logger
import stable_baselines3.common.monitor
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
Notes:
    A command to kill all Python processes within the GPU.
    $ ../>  sudo killall -9 python

    Start training the model.
    $ ../>  python train_sac.py
"""

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
#   'SAC':
#       Soft Actor Critic (SAC) Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor.
#   'SAC_HER':
#       Soft Actor Critic (SAC) + Hindsight Experience Replay (HER)
CONST_ALGORITHM = 'SAC'
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program designed to train a specific robotic arm for a reach task in a pre-defined environment, utilizing 
        the Soft Actor-Critic (SAC) reinforcement learning algorithm.

        The algorithm can be extended using the Hindsight Experience Replay (HER).

        More information about Stable Baselines3 (SB3) can be found at the link below:
            https://stable-baselines3.readthedocs.io/en/master/
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The specified path of the file to save the log file.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Training/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}'

    # Removes old files (if any) created by the previous training.
    #   Training progress.
    for _, file_name in enumerate(['monitor.csv', 'progress.csv', 'time.txt']):
        if os.path.isfile(f'{file_path}/{file_name}'):
            os.remove(f'{file_path}/{file_name}')
            print(f'[INFO] The file has been successfully removed.')
            print(f'[INFO] >> {file_path}/{file_name}')
    #   Model.
    if os.path.isfile(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model.zip'):
        os.remove(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model.zip')
        print(f'[INFO] The file has been successfully removed.')
        print(f'[INFO] >> {CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model.zip')

    # Configure the logger for the current training process.
    logger_cfg = stable_baselines3.common.logger.configure(file_path, ['stdout', 'csv'])

    # Create the environment that was previously registered using gymnasium.register() within the __init__.py file.
    #   More information can be found in the following script:
    #       ../src/Industrial_Robotics_Gym/__init__.py
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_ENV_MODE))

    # A monitor for a defined gym environment that is used to collect data such as reward, length and time.
    gym_environment = stable_baselines3.common.monitor.Monitor(gym_environment, file_path)

    # Create a vectorized environment.
    gym_environment = stable_baselines3.common.vec_env.DummyVecEnv([lambda: gym_environment])

    # The normal (Gaussian) action noise with a specified mean and standard deviation.
    n_action = gym_environment.action_space.shape[-1]
    action_noise = stable_baselines3.common.noise.NormalActionNoise(mean=np.zeros(n_action), sigma=0.1*np.ones(n_action))

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    if CONST_ALGORITHM == 'SAC':
        model = stable_baselines3.SAC(policy="MultiInputPolicy", env=gym_environment, gamma=0.95, learning_rate=0.001, action_noise=action_noise, device='cuda', 
                                      batch_size=256, policy_kwargs=dict(net_arch=[256, 256, 256]), verbose=1)
    elif CONST_ALGORITHM == 'SAC_HER':
        model = stable_baselines3.SAC(policy="MultiInputPolicy", env=gym_environment, replay_buffer_class=stable_baselines3.HerReplayBuffer, gamma=0.95, learning_rate=0.001, action_noise=action_noise, device='cuda', 
                                      batch_size=256, replay_buffer_kwargs={'goal_selection_strategy' : 'future', 'n_sampled_goal' : 4}, policy_kwargs={'net_arch' : [256, 256, 256], 'n_critics' : 2}, verbose=1)

    # Set the logger configuration for the training.  
    model.set_logger(logger_cfg)
    
    # Train the model during the selected time steps.
    model.learn(total_timesteps=100000, log_interval=10)

    # Save all the attributes of the object and the model parameters in a zip-file.
    model.save(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/model')

    # Save the training time to a file.
    print(f'[INFO] Training time: {time.time() - t_0:0.05f} in seconds.')
    RoLE.Utilities.File_IO.Save(f'{file_path}/time', f'Training time: {time.time() - t_0:0.05f} in seconds.', 'txt', '')

    # Disconnect the created environment from a physical server.
    gym_environment.close()
    
if __name__ == '__main__':
    main()
