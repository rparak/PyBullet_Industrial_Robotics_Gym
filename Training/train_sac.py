# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../' + 'src')
# Time (Time access and conversions)
import time
# OS (Operating system interfaces)
import os
# Shutil (High-level file operations)
import shutil
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
# ...
CONST_ENV_MODE = 'Default'
# ...
#   SAC, SAC_HER
CONST_ALGORITHM_NAME = 'SAC'
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The specified path of the file to save the log file.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Training/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM_NAME}/{Robot_Str.Name}'

    # Removes old files (if any) created by the previous training.
    #   Training progress.
    for _, file_name in enumerate(['monitor', 'progress']):
        if os.path.isfile(f'{file_path}/{file_name}.csv'):
            os.remove(f'{file_path}/{file_name}.csv')
            print(f'[INFO] The file has been successfully removed.')
            print(f'[INFO] >> {file_path}/{file_name}.csv')
    #   Model.
    if os.path.isfile(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM_NAME}/{Robot_Str.Name}/model.zip'):
        os.remove(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM_NAME}/{Robot_Str.Name}/model.zip')
        print(f'[INFO] The file has been successfully removed.')
        print(f'[INFO] >> {CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM_NAME}/{Robot_Str.Name}/model.zip')

    # ...
    logger_cfg = stable_baselines3.common.logger.configure(file_path, ['stdout', 'csv'])

    # ...
    gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_ENV_MODE))

    # ...
    gym_environment = stable_baselines3.common.monitor.Monitor(gym_environment, file_path)
    gym_environment = stable_baselines3.common.vec_env.DummyVecEnv([lambda: gym_environment])

    print('[INFO] The calculation is in progress.')
    t_0 = time.time()

    # ...
    if CONST_ALGORITHM_NAME == 'SAC':
        model = stable_baselines3.SAC(policy="MultiInputPolicy", env=gym_environment, gamma=0.95, learning_rate=0.001, device='cuda', 
                                      batch_size=256, policy_kwargs=dict(net_arch=[256, 256, 256]), verbose=1)
    elif CONST_ALGORITHM_NAME == 'SAC_HER':
        model = stable_baselines3.SAC(policy="MultiInputPolicy", env=gym_environment, replay_buffer_class=stable_baselines3.HerReplayBuffer, gamma=0.95, learning_rate=0.001, device='cuda', 
                                      batch_size=256, replay_buffer_kwargs={'goal_selection_strategy' : 'future', 'n_sampled_goal' : 4}, policy_kwargs={'net_arch' : [256, 256, 256], 'n_critics' : 2}, verbose=1)

    # ...  
    model.set_logger(logger_cfg)
    
    # ...
    model.learn(total_timesteps=10, log_interval=10)

    # ...
    model.save(f'{CONST_PROJECT_FOLDER}/Data/Model/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM_NAME}/{Robot_Str.Name}/model')

    # ...
    print(f'[INFO] Time: {time.time() - t_0:0.05f} in seconds.')

if __name__ == '__main__':
    main()
