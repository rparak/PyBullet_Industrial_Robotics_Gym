# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../../' + 'src')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO

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
# The name of the reinforcement learning algorithm. 
#   Deep Deterministic Policy Gradient (DDPG)
#       CONST_ALGORITHM = 'DDPG' or 'DDPG_HER'
#   Soft Actor Critic (SAC)
#       CONST_ALGORITHM = 'SAC' or 'SAC_HER'
#   Twin Delayed DDPG (TD3)
#       CONST_ALGORITHM = 'TD3' or 'TD3_HER'
CONST_ALGORITHM = 'DDPG_HER'
# Number of randomly generated targets.
CONST_N_TARGETS = 100
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to show result data from the prediction. The metrics, such as Success Rate, Reward per Episode, Episode Length, and Absolute 
        Position Error (APE), were used to evaluate the performance of the selected reinforcement learning algorithm.

        In this case, the target is randomly defined and we observe the metrics defined on top.
        
        The program simply displays the results as the values shown in the console.
        
        More information about the prediction process can be found in the script below:
            ../Random/train_{CONST_ALGORITHM}.py
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file was saved.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/random_N_{CONST_N_TARGETS}'

    # Read data from the file.
    data = RoLE.Utilities.File_IO.Load(file_path, 'txt', ',')

    # Store the 'Success Rate' data.
    s_r = data[:, 1]

    # Find indexes where the targat has not been successfully found.
    index = np.where(s_r < 1.0)[0]

    print(f'[INFO] Algorithm: {CONST_ALGORITHM}')
    print(f'[INFO] Environment mode: {CONST_ENV_MODE}')
    print(f'[INFO] Number of targets unsuccessfully found: {index.size}')
    label = [r'Success Rate', r'Reward per Episode', r'Episode Length', r'Absolute Position Error (APE)']
    for i, label_i in enumerate(label):
        # Display the results as the values shown in the console.
        print(f'[INFO] Metrics: {label_i}')
        if index.size == 0 or label_i == 'Success Rate':
            print(f'[INFO] >> max = {np.max(data[:, i + 1])}')
            print(f'[INFO] >> min = {np.min(data[:, i + 1])}')
            print(f'[INFO] >> mean = {np.mean(data[:, i + 1])}')
        else:
            metrics_data_tmp = []
            for j, data_i in enumerate(data[:, i + 1]):
                if j not in index:
                    metrics_data_tmp.append(data_i)
            print(f'[INFO] >> max = {np.max(metrics_data_tmp)}')
            print(f'[INFO] >> min = {np.min(metrics_data_tmp)}')
            print(f'[INFO] >> mean = {np.sum(metrics_data_tmp)/CONST_N_TARGETS}')

if __name__ == '__main__':
    sys.exit(main())