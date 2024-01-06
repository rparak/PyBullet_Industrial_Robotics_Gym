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
CONST_ALGORITHM = 'DDPG'
# Number of randomly generated targets.
CONST_N_TARGETS = 100
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        ...
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The name of the path where the file was saved.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/random_N_{CONST_N_TARGETS}'

    # Read data from the file.
    data = RoLE.Utilities.File_IO.Load(file_path, 'txt', ',')

    # ... add success rate vector to calculate min, max, wihout unsucessfull 
    
    print(f'[INFO] Algorithm: {CONST_ALGORITHM}')
    print(f'[INFO] Environment mode: {CONST_ENV_MODE}')
    label = [r'Success Rate', r'Reward per Episode', r'Episode Length', r'$e_{p}(t)$']
    for i, label_i in enumerate(label):
        # Display the results as the values shown in the console.
        print(f'[INFO] Metrics: {label_i}')
        print(f'[INFO] >> max = {np.min(np.abs(data[:, i + 1]))}')
        print(f'[INFO] >> min = {np.min(np.abs(data[:, i + 1]))}')
        print(f'[INFO] >> mean = {np.mean(data[:, i + 1])}')

if __name__ == '__main__':
    sys.exit(main())