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
    """
    Description:
        A program to compare the metrics obtained during prediction of a 'reach' task in a pre-defined environment, utilizing 
        a specific reinforcement learning algorithm.

        In this case, the target is randomly defined and we observe the metrics as Success Rate, Reward per Episode, Episode Length, and Absolute 
        Position Error (APE).

        The program visualizes the results in a graph (plot).

        Note:
            The comparison is only defined for the Universal Robots UR3 robotic arm. The other 
            robotic arms are trained using the best method obtained from the comparison.

        More information about the prediction process can be found in the script below:
            ../Random/train_{CONST_ALGORITHM}.py
    """

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

    print(f'[INFO] Environment mode: {CONST_ENV_MODE}')
    s_r = []; r = []; l = []; e = []
    for i, data_i in enumerate(data):
        s_r_tmp = []; r_tmp = []; l_tmp = []; e_tmp = []
        for _, data_ij in enumerate(data_i):
            s_r_tmp.append(data_ij[1]); 
            if data_ij[1] == 1.0:
                r_tmp.append(data_ij[2]); l_tmp.append(data_ij[3]); 
                e_tmp.append(data_ij[4])

        # Display informations.
        print(f'[INFO] Algorithm: {CONST_ALGORITHMS[i]}')
        for _, (metrics_data_i, metrics_label_i) in enumerate(zip([s_r_tmp, r_tmp, l_tmp, e_tmp], 
                                                                  [r'Success Rate', r'Reward per Episode', 
                                                                   r'Episode Length', r'Absolute Position Error (APE)'])):
            print(f'[INFO] Metrics: {metrics_label_i}')
            print(f'[INFO] >> mean = {np.sum(metrics_data_i)/CONST_N_TARGETS}')

        # Store the data.
        s_r.append(s_r_tmp); r.append(r_tmp); l.append(l_tmp)
        e.append(e_tmp)    

    # Set the parameters for the scientific style.
    plt.style.use('science')

    label_algorithm = ['DDPG', 'DDPG + HER', 'SAC', 'SAC + HER', 'TD3', 'TD3 + HER']
    label = [r'Success Rate', r'Reward per Episode', r'Episode Length', r'Absolute Position Error (APE)']
    for i, (data_i, label_i) in enumerate(zip([s_r, r, l, e], label)):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        box_plot_out = ax.boxplot(data_i, labels=label_algorithm, showmeans=True, patch_artist=True, meanline=True, medianprops=dict(linestyle=None, linewidth=0.0),
                                  showfliers=False)

        # Set the properties of the box plot.
        #   Boxes.
        plt.setp(box_plot_out['boxes'][0], color='#f2c89b', facecolor='#ffffff'); plt.setp(box_plot_out['boxes'][1], color='#e69138', facecolor='#ffffff')
        plt.setp(box_plot_out['boxes'][2], color='#c5d3e2', facecolor='#ffffff'); plt.setp(box_plot_out['boxes'][3], color='#8ca8c5', facecolor='#ffffff')
        plt.setp(box_plot_out['boxes'][4], color='#d2a6bc', facecolor='#ffffff'); plt.setp(box_plot_out['boxes'][5], color='#a64d79', facecolor='#ffffff')
        #   Whiskers.
        plt.setp(box_plot_out['whiskers'][0], color='#f2c89b'); plt.setp(box_plot_out['whiskers'][1], color='#f2c89b')
        plt.setp(box_plot_out['whiskers'][2], color='#e69138'); plt.setp(box_plot_out['whiskers'][3], color='#e69138')
        plt.setp(box_plot_out['whiskers'][4], color='#c5d3e2'); plt.setp(box_plot_out['whiskers'][5], color='#c5d3e2')
        plt.setp(box_plot_out['whiskers'][6], color='#8ca8c5'); plt.setp(box_plot_out['whiskers'][7], color='#8ca8c5')
        plt.setp(box_plot_out['whiskers'][8], color='#d2a6bc'); plt.setp(box_plot_out['whiskers'][9], color='#d2a6bc')
        plt.setp(box_plot_out['whiskers'][10], color='#a64d79'); plt.setp(box_plot_out['whiskers'][11], color='#a64d79')
        #   Means.
        plt.setp(box_plot_out['means'][0], color='#f2c89b'); plt.setp(box_plot_out['means'][1], color='#e69138')
        plt.setp(box_plot_out['means'][2], color='#c5d3e2'); plt.setp(box_plot_out['means'][3], color='#8ca8c5')
        plt.setp(box_plot_out['means'][4], color='#d2a6bc'); plt.setp(box_plot_out['means'][5], color='#a64d79')
        #   Caps.
        plt.setp(box_plot_out['caps'][0], color='#f2c89b'); plt.setp(box_plot_out['caps'][1], color='#f2c89b')
        plt.setp(box_plot_out['caps'][2], color='#e69138'); plt.setp(box_plot_out['caps'][3], color='#e69138')
        plt.setp(box_plot_out['caps'][4], color='#c5d3e2'); plt.setp(box_plot_out['caps'][5], color='#c5d3e2')
        plt.setp(box_plot_out['caps'][6], color='#8ca8c5'); plt.setp(box_plot_out['caps'][7], color='#8ca8c5')
        plt.setp(box_plot_out['caps'][8], color='#d2a6bc'); plt.setp(box_plot_out['caps'][9], color='#d2a6bc')
        plt.setp(box_plot_out['caps'][10], color='#a64d79'); plt.setp(box_plot_out['caps'][11], color='#a64d79')

        # Set parameters of the graph (plot).
        #ax.set_title(f'...', fontsize=25, pad=25.0)
        #   Label
        ax.set_xlabel(r'Deep Reinforcement Learning Algorithm', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{label_i}', fontsize=15, labelpad=10) 
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