# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# OS (Operating system interfaces)
import os
# Pandas (Data analysis and manipulation) [pip3 install pandas]
import pandas as pd
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters

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
#   Deep Deterministic Policy Gradient (DDPG)
#       CONST_ALGORITHM = 'DDPG' or 'DDPG_HER'
#   Soft Actor Critic (SAC)
#       CONST_ALGORITHM = 'SAC' or 'SAC_HER'
#   Twin Delayed DDPG (TD3)
#       CONST_ALGORITHM = 'TD3' or 'TD3_HER'
CONST_ALGORITHMS = ['DDPG', 'DDPG_HER', 'SAC', 'SAC_HER', 
                    'TD3', 'TD3_HER']
# The selected metrics that will be observed for the comparison.
CONST_METRICS = ['rollout/success_rate', 'rollout/ep_rew_mean', 'rollout/ep_len_mean',
                 'train/critic_loss', 'train/actor_loss']
# Minimum success rate.
CONST_MIN_SUCCESS_RATE = 0.99
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to compare the metrics obtained during training for a 'reach' task in a pre-defined 
        environment, utilizing a specific reinforcement learning algorithm

        The program simply displays the results as the values shown in the console.
        
        Note:
            The comparison is only defined for the Universal Robots UR3 robotic arm. The other 
            robotic arms are trained using the best method obtained from the comparison.

        More information about the training process can be found in the script below:
            ../PyBullet_Industrial_Robotics_Gym/Training/train_{CONST_ALGORITHM}.py
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    data = []
    for _, name_algorithm in enumerate(CONST_ALGORITHMS):
        # The specified path of the file to save the log file.
        file_path = f'{CONST_PROJECT_FOLDER}/Data/Training/Environment_{CONST_ENV_MODE}/{name_algorithm}/{Robot_Str.Name}'

        # Read data from the file {*.csv}.
        if os.path.isfile(f'{file_path}/progress.csv'):
            data.append(pd.read_csv(f'{file_path}/progress.csv'))
        else:
            print('[WARNING] The file does not exist.')
            print(f'[WARNING] >> {file_path}/progress.csv')
            exit(0)

    # Display the results as the values shown in the console.
    for _, (name_algorithm, data_i) in enumerate(zip(CONST_ALGORITHMS, data)):
        print(f'[INFO] {name_algorithm}')
        for _, metric_i in enumerate(CONST_METRICS):
            if 'success_rate' in metric_i:
                #sucess_rate = np.max(data_i[metric_i])
                info = np.where(data_i[metric_i] >= CONST_MIN_SUCCESS_RATE); index = np.min(info[0])
                print(f'[INFO] >> First successful result in a timestep: {data_i["time/total_timesteps"][index]}')
                print(f'[INFO] >> Percentage of success with a defined minimum success rate: {(info[0].size / data_i[metric_i].size)}')
            else:
                if metric_i in ['rollout/ep_rew_mean', 'rollout/ep_len_mean']:
                    print(f'[INFO] >> mean({metric_i}) - success: {np.mean(data_i[metric_i][index::])}')
                else:
                    print(f'[INFO] >> min({metric_i}): {np.min(np.abs(data_i[metric_i]))}')

if __name__ == '__main__':
    sys.exit(main())