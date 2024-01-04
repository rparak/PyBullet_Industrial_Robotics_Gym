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
# Number of randomly generated targets.
CONST_N_TARGETS = 100
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    data = []
    for _, name_algorithm in enumerate(CONST_ALGORITHMS):
        # The name of the path where the file was saved.
        file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{name_algorithm}/{Robot_Str.Name}/random_N_{CONST_N_TARGETS}'

        # Read data from the file {*.txt}.
        if os.path.isfile(f'{file_path}.txt'):
            data.append(RoLE.Utilities.File_IO.Load(file_path, 'txt', ','))
        else:
            print('[WARNING] The file does not exist.')
            print(f'[WARNING] >> {file_path}.txt')
            exit(0)

if __name__ == '__main__':
    sys.exit(main())