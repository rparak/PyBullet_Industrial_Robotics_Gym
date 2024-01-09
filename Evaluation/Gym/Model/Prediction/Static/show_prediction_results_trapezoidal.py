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
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
#       ../RoLE/Trajectory/Core
import RoLE.Trajectory.Core
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
CONST_ALGORITHM = 'DDPG'
# B-Spline interpolation parameters.
#   n: Degree of a polynomial.
#   N: The number of points to be generated in the interpolation function.
#   'method': The method to be used to select the parameters of the knot vector. 
#               method = 'Uniformly-Spaced', 'Chord-Length' or 'Centripetal'.
CONST_B_SPLINE = {'n': 3, 'N': 100, 'method': 'Chord-Length'}
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to show result data from the prediction. The absolute position error (APE) metric was used to evaluate the performance 
        of the selected reinforcement learning algorithm.

        In this case, the target is statically defined and we observe the predicted points of the path and absolute position error (APE).
        
        The program visualizes the results in a graph (plot).
        
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
        
    # The name of the path where the file was saved.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/path_static_target'

    # Read data from the file.
    data = RoLE.Utilities.File_IO.Load(file_path, 'txt', ',')
    
    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Display TCP(Tool Center Point) parameters.
    y_label = [r'$x(t)$ in meters', r'$y(t)$ in meters', r'$z(t)$ in meters']
    for i, (data_i, p_i) in enumerate(zip(data.T, T.p.all())):
        # Initialization of multi-segment constraints for trajectory generation.
        #  1\ Input control points (waypoints) to be used for trajectory generation.
        P_i = data_i.copy()
        #  2\ Trajectory duration between control points.
        delta_T = np.array((P_i.size - 1) * [2.5], dtype=np.float32)
        #  3\ Duration of the blend phase.
        t_blend = np.array(P_i.size * [0.5], dtype=np.float32)

        # Initialization of the class to generate multi-segment trajectory.
        MST_Cls = RoLE.Trajectory.Core.Multi_Segment_Cls('Trapezoidal', delta_time=0.1)

        # Generation of position multi-segment trajectories from input parameters.
        (s_i, _, _, T_i, L_i) = MST_Cls.Generate(P_i, delta_T, t_blend)

        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        """
        ax.scatter(MST_Cls.t[-1], p_i, c='#84b070', s=100.0, linewidths=1, label=f'Target')
        ax.plot(T_i, P_i, 'o--', color='#d0d0d0', linewidth=1.0, markersize=6.0, 
                markeredgewidth=3.0, markerfacecolor='#ffffff', label=f'Predicted Control Points')
        ax.plot(MST_Cls.t, s_i, '-', color='#ffcb99', linewidth=1.0, label=f'Trapezoidal Trajectory (L = {L_i:.03})')
        ax.plot([MST_Cls.t[-1]] * 2, [P_i[-1], p_i], '.-', color='#e06666', linewidth=1.0, label=f'$\Delta$e = {np.abs(data_i[-1] - p_i):.5f} in meters')
        # Additional lines.
        ax.plot([MST_Cls.t[0], T_i[0]], [s_i[0], P_i[0]], '--', color='#d0d0d0', linewidth=1.0)
        ax.plot([MST_Cls.t[-1], T_i[-1]], [s_i[-1], P_i[-1]], '--', color='#d0d0d0', linewidth=1.0)
        """

        print(T_i)
        ax.plot(MST_Cls.t * (1 / np.max(MST_Cls.t)), s_i, '-', color='#ffcb99', linewidth=1.0, label=f'Trapezoidal Trajectory (L = {L_i:.03})')

        # Set parameters of the graph (plot).
        #   Set the x ticks.
        #ax.set_xticks(np.arange(np.min(MST_Cls.t) - 2.0, np.max(MST_Cls.t) + 2.0, 0.5))
        #   Set the y ticks.
        tick_y_tmp = (np.max(data_i) - np.min(data_i))/10.0
        tick_y = tick_y_tmp if tick_y_tmp != 0.0 else 0.1
        ax.set_yticks(np.arange(np.min(data_i) - tick_y, np.max(data_i) + tick_y, tick_y))
        #   Label.
        ax.set_xlabel(r't in seconds', fontsize=15, labelpad=10)
        ax.set_ylabel(f'{y_label[i]}', fontsize=15, labelpad=10) 
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