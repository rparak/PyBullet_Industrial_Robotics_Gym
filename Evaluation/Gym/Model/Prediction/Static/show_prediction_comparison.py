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
#   ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
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
#   Deep Deterministic Policy Gradient (DDPG)
#       CONST_ALGORITHM = 'DDPG' or 'DDPG_HER'
#   Soft Actor Critic (SAC)
#       CONST_ALGORITHM = 'SAC' or 'SAC_HER'
#   Twin Delayed DDPG (TD3)
#       CONST_ALGORITHM = 'TD3' or 'TD3_HER'
CONST_ALGORITHMS = ['DDPG', 'DDPG_HER', 'SAC', 'SAC_HER', 
                    'TD3', 'TD3_HER']
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to compare the metrics obtained during prediction of a 'reach' task in a pre-defined environment, utilizing 
        a specific reinforcement learning algorithm.

        In this case, the target is statically defined and we observe the predicted points of the path and absolute 
        position error (APE). The path will be interpolated using B-Spline.

        The program visualizes the results in a graph (plot).
        
        Note:
            The comparison is only defined for the Universal Robots UR3 robotic arm. The other 
            robotic arms are trained using the best method obtained from the comparison.

        More information about the prediction process can be found in the script below:
            ../Static/train_{CONST_ALGORITHM}.py
    """
        
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the structure of the main parameters of the environment for the defined robotic arm.
    Env_Structure = PyBullet.Utilities.Get_Environment_Structure(Robot_Str.Name, 0 if CONST_ENV_MODE == 'Default' else 1)

    # Create a static target that was used to predict the path.
    v = np.array([Env_Structure.C.Target.T.p.x + (Env_Structure.C.Target.Size[0]/4.0), 
                  Env_Structure.C.Target.T.p.y + (-1) * (Env_Structure.C.Target.Size[1]/4.0), 
                  Env_Structure.C.Target.T.p.z], dtype=np.float64)
    T = HTM_Cls(None, np.float64).Rotation(Env_Structure.C.Target.T.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation(v)

    data = []
    for _, name_algorithm in enumerate(CONST_ALGORITHMS):
        # The name of the path where the file was saved.
        file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{name_algorithm}/{Robot_Str.Name}/path_static_target'

        # Read data from the file {*.txt}.
        if os.path.isfile(f'{file_path}.txt'):
            data.append(RoLE.Utilities.File_IO.Load(file_path, 'txt', ','))
        else:
            print('[WARNING] The file does not exist.')
            print(f'[WARNING] >> {file_path}.txt')
            exit(0)

    x = []; y = []; z = []
    for i, data_i in enumerate(data):
        x_tmp = []; y_tmp = []; z_tmp = []
        for _, data_ij in enumerate(data_i):
            
            x_tmp.append(data_ij[0]); y_tmp.append(data_ij[1]); 
            z_tmp.append(data_ij[2])
        x.append(x_tmp); y.append(y_tmp); z.append(z_tmp)          
    
        # Display informations.
        print(f'[INFO] Absolute Position Error (APE): {CONST_ALGORITHMS[i]}')
        print(f'[INFO] >> e_p(t) = {np.linalg.norm([x_tmp[-1], y_tmp[-1], z_tmp[-1]] - T.p.all(), axis=-1).astype(np.float32):.5f} in meters')

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Display TCP(Tool Center Point) parameters.
    label = [['DDPG', 'SAC', 'TD3'], 
             ['DDPG + HER', 'SAC + HER', 'TD3 + HER']]
    color = [['#f2c89b', '#c5d3e2', '#d2a6bc'], 
             ['#e69138', '#8ca8c5', '#a64d79']]
    y_label = [r'$x(\hat{t})$ in meters', r'$y(\hat{t})$ in meters', r'$z(\hat{t})$ in meters']; iteration = [0, 2, 3, 1, 4, 5]
    for _, (y_label_i, p_achieved_i, p_desired_i) in enumerate(zip(y_label, [x, y, z], T.p.all())):
        # Create a figure.
        _, ax = plt.subplots(1, 2)

        print(f'[INFO] Absolute Error: {y_label_i[1]} - axis')
        for i, (color_i, label_i) in enumerate(zip(color, label)):
            p_min = []; p_max = []
            for _, (color_ij, label_ij, p_achieved_ij) in enumerate(zip(color_i, label_i, p_achieved_i[i::2])):
                # Get the normalized time.
                t_hat = np.linspace(0.0, 1.0, len(p_achieved_ij))

                # Visualization of relevant structures.
                ax[i].plot(t_hat, p_achieved_ij, 'o--', color=color_ij, linewidth=1.0, markersize=6.0, 
                           markeredgewidth=3.0, markerfacecolor='#ffffff', label=label_ij)
                
                # Display informations.
                print(f'[INFO] {label_ij}:')
                print(f'[INFO] >> delta_e = {np.abs(p_achieved_ij[-1] - p_desired_i):.5f}')

                # Get the minimum and maximum position.
                p_min.append(np.min(p_achieved_ij)); p_max.append(np.max(p_achieved_ij))

            # Visualization of relevant structures.
            ax[i].scatter(t_hat[-1], p_desired_i, c='#84b070', s=100.0, linewidths=1, label=f'Target')

            # Set parameters of the graph (plot).
            #   Set the x ticks.
            ax[i].set_xticks(np.arange(np.min(t_hat) - 0.1, np.max(t_hat) + 0.1, 0.1))
            #   Set the y ticks.
            tick_y_tmp = (np.max(p_max) - np.min(p_min))/10.0
            tick_y = tick_y_tmp if tick_y_tmp != 0.0 else 0.1
            ax[i].set_yticks(np.arange(np.min(p_min) - tick_y, np.max(p_max) + tick_y, tick_y))
            #   Label.
            ax[i].set_xlabel(r'Normalized time $\hat{t}$ in the range of [0.0, 1.0]', fontsize=15, labelpad=10)
            ax[i].set_ylabel(f'{y_label_i}', fontsize=15, labelpad=10) 
            #   Set parameters of the visualization.
            ax[i].grid(which='major', linewidth = 0.15, linestyle = '--')
            # Show the labels (legends) of the graph.
            ax[i].legend(fontsize=10.0)

        # Show the result.
        plt.show()

if __name__ == '__main__':
    sys.exit(main())