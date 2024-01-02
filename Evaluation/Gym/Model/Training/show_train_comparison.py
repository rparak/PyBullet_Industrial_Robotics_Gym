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
CONST_ALGORITHMS = ['DDPG', 'DDPG_HER', 'SAC', 'SAC_HER', 
                    'TD3', 'TD3_HER']
# The selected metric to be displayed in the graph (plot).
#   CONST_METRIC = 'rollout/success_rate', 'rollout/ep_rew_mean', 'rollout/ep_len_mean', 
#                  'train/actor_loss' or 'train/critic_loss'
CONST_METRIC = 'rollout/success_rate'
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to compare the metrics obtained during training for a 'reach' task in a pre-defined 
        environment, utilizing a specific reinforcement learning algorithm

        The program visualizes the results in a graph (plot).

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

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Create a figure.
    _, ax = plt.subplots()

    # Get the x and y axis of the graph.
    color = ['#f2c89b', '#e69138', '#c5d3e2', '#8ca8c5', 
             '#d2a6bc', '#a64d79']
    for _, (color_i, data_i, label_i) in enumerate(zip(color, data, ['DDPG', 'DDPG + HER', 'SAC', 'SAC + HER', 
                                                                     'TD3', 'TD3 + HER'])):
        # Visualization of relevant structures.
        ax.plot(data_i['time/total_timesteps'], data_i[CONST_METRIC], '-', color=color_i, 
                label=label_i)

    # Set parameters of the graph (plot).
    #ax.set_title(f'Title ...', fontsize=25, pad=25.0)
    #   Set the x ticks.
    ax.set_xticks(np.arange(0.0, 100000.0 + 10000.0, 10000.0))
    #   Label
    ax.set_xlabel(r'Deep Reinforcement Learning Algorithm', fontsize=15, labelpad=10)
    if CONST_METRIC == 'rollout/success_rate':
        y_label = 'Mean Success Rate During Training'
    elif CONST_METRIC == 'rollout/ep_rew_mean':
        y_label = 'Mean Training Reward per Episode'
    elif CONST_METRIC == 'rollout/ep_len_mean':
        y_label = 'Mean Episode Length'
    elif CONST_METRIC == 'train/actor_loss':
        y_label = 'Actor Loss'
    elif CONST_METRIC == 'train/critic_loss':
        y_label = 'Critic Loss'
    ax.set_ylabel(y_label, fontsize=15, labelpad=10) 
    #   Set parameters of the visualization.
    ax.grid(which='major', linewidth = 0.15, linestyle = '--')
    # Get handles and labels for the legend.
    handles, labels = plt.gca().get_legend_handles_labels()
    # Remove duplicate labels.
    legend = dict(zip(labels, handles))
    # Show the labels (legends) of the graph.
    ax.legend(legend.values(), legend.keys(), fontsize=10.0)

    # Show the result.
    plt.show()

if __name__ == '__main__':
    sys.exit(main())