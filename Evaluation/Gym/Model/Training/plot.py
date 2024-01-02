# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../../' + 'src' not in sys.path:
    sys.path.append('../../../../' + 'src')
# OS (Operating system interfaces)
import os
# Pandas (Data analysis and manipulation) [pip3 install pandas]
import pandas as pd
# SciencePlots (Matplotlib styles for scientific plotting) [pip3 install SciencePlots]
import scienceplots
# Matplotlib (Visualization) [pip3 install matplotlib]
import matplotlib.pyplot as plt
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
CONST_ALGORITHM = 'TD3_HER'
# The selected metric to be displayed in the graph (plot).
CONST_METRIC = ''
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        ...
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # The specified path of the file to save the log file.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Training/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}'

    # Read data from the file {*.csv}.
    if os.path.isfile(f'{file_path}/progress.csv'):
        data = pd.read_csv(f'{file_path}/progress.csv')
    else:
        print('[WARNING] The file does not exist.')
        exit(0)

    # Set the parameters for the scientific style.
    plt.style.use('science')

if __name__ == '__main__':
    sys.exit(main())