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
#   ../RoLE/Interpolation/B_Spline/Core
import RoLE.Interpolation.B_Spline.Core as B_Spline
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
CONST_ENV_MODE = 'Collision-Free'
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
# Number of optimized control points.
CONST_NUM_OF_OPT_POINTS = 4
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        A program to show result data from the prediction. The absolute position error (APE) metric was used to evaluate the performance 
        of the selected reinforcement learning algorithm.

        In this case, the target is statically defined and we observe the predicted points of the path and absolute 
        position error (APE). The predicted points are optimized using the least squares method and interpolated 
        using B-Spline.
        
        The program visualizes the results in a graph (plot).
        
        More information about the prediction process can be found in the script below:
            ../Static/predict_{CONST_ALGORITHM}.py
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
    
    # Initialization of a specific class to work with B-Spline curves.
    S_Cls = B_Spline.B_Spline_Cls(CONST_B_SPLINE['n'], CONST_B_SPLINE['method'], data, 
                                  CONST_B_SPLINE['N'])
    
    # Interpolation of parametric B-Spline curve.
    S = S_Cls.Interpolate()

    # Obtain the arc length L(x) of the general parametric curve.
    L = S_Cls.Get_Arc_Length()

    # Initialization of a specific class to work with B-Spline curves from 
    # control points.
    S_Cls_2 = B_Spline.B_Spline_Cls(CONST_B_SPLINE['n'], CONST_B_SPLINE['method'], data, 
                                    CONST_B_SPLINE['N'])
    
    # Get optimized control points.
    S_Cls_optimized = S_Cls_2.Optimize_Control_Points(CONST_NUM_OF_OPT_POINTS)

    # Interpolation of the parametric B-Spline curve from the optimized control points.
    S_optimized = S_Cls_optimized.Interpolate()

    # Obtain the arc length L(x) of the optimized parametric curve.
    L_optimized = S_Cls_optimized.Get_Arc_Length()

    # Display informations.
    print('[INFO] Absolute Position Error (APE):')
    print(f'[INFO] >> e_p(t) = {np.linalg.norm(data[-1] - T.p.all(), axis=-1).astype(np.float32):.5f} in meters')
    print('[INFO] Arc length L(t):')
    print(f'[INFO] >> L = {L:.5f} in meters')
    print(f'[INFO] >> L_optimized = {L_optimized:.5f} in meters')

    # Set the parameters for the scientific style.
    plt.style.use('science')

    # Get the normalized time.
    t_hat = np.linspace(0.0, 1.0, data[:, 0].size)
    t_hat_optimized = np.linspace(0.0, 1.0, S_Cls_optimized.P[:, 0].size)

    # Display TCP(Tool Center Point) parameters.
    y_label = [r'$x(\hat{t})$ in meters', r'$y(\hat{t})$ in meters', r'$z(\hat{t})$ in meters']
    for i, (P_i, S_i, O_P_i, O_S_i, p_i) in enumerate(zip(S_Cls.P.T, S.T, S_Cls_optimized.P.T, S_optimized.T, T.p.all())):
        # Create a figure.
        _, ax = plt.subplots()

        # Visualization of relevant structures.
        ax.scatter(t_hat[-1], p_i, c='#84b070', s=100.0, linewidths=1, label=f'Target')
        ax.plot(t_hat, P_i, 'o--', color='#d0d0d0', linewidth=1.0, markersize=6.0, 
                markeredgewidth=3.0, markerfacecolor='#ffffff', label=f'Control Points')
        ax.plot(S_Cls.x, S_i, '-', color='#d0d0d0', linewidth=1.0, label=f'B-Spline (L = {L:.03})')
        ax.plot(t_hat_optimized, O_P_i, 'o--', color='#ffcb99', linewidth=1.0, markersize=6.0, 
                markeredgewidth=3.0, markerfacecolor='#ffffff', label=f'Optimized Control Points using the Least-Squares Method')
        ax.plot(S_Cls.x, O_S_i, '-', color='#ffcb99', linewidth=1.0, label=f'Optimized B-Spline (L = {L_optimized:.03})')
        ax.plot([t_hat[-1]] * 2, [P_i[-1], p_i], '.-', color='#e06666', linewidth=1.0, label=f'$\Delta$e = {np.abs(P_i[-1] - p_i):.5f} in meters')

        # Set parameters of the graph (plot).
        #   Set the x ticks.
        ax.set_xticks(np.arange(np.min(t_hat) - 0.1, np.max(t_hat) + 0.1, 0.1))
        #   Set the y ticks.
        tick_y_tmp = (np.max(O_P_i) - np.min(O_P_i))/10.0
        tick_y = tick_y_tmp if tick_y_tmp != 0.0 else 0.1
        ax.set_yticks(np.arange(np.min(O_P_i) - tick_y, np.max(O_P_i) + tick_y, tick_y))
        #   Label.
        ax.set_xlabel(r'Normalized time $\hat{t}$ in the range of [0.0, 1.0]', fontsize=15, labelpad=10)
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