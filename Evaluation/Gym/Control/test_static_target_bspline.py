# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# OS (Operating system interfaces)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
#       ../RoLE/Interpolation/B_Spline/Core
import RoLE.Interpolation.B_Spline.Core as B_Spline
#   PyBullet
#       ../PyBullet/Utilities
import PyBullet.Utilities
#       ../PyBullet/Core
import PyBullet.Core

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Numerical IK Parameters.
#   The properties of the inverse kinematics solver.
CONST_IK_PROPERTIES = {'delta_time': None, 'num_of_iteration': 500, 
                       'tolerance': 1e-30}
# Visibility of the target position as the 'ghost' of the robotic model.
CONST_VISIBILITY_GHOST = False
# The name of the environment mode.
#   'Default': 
#       The mode called "Default" demonstrates an environment without a collision object.
#   'Collision-Free': 
#       The mode called "Collision-Free" demonstrates an environment with a collision object.
CONST_ENV_MODE = 'Default'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
if CONST_ENV_MODE == 'Default':
    CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': True, 'fps': 100, 
                                     'External_Base': None, 'Env_ID': 0,
                                     'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                                'Position': [0.05, -0.10, 0.06]}}
else:
    CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': True, 'fps': 100, 
                                     'External_Base': None, 'Env_ID': 1,
                                     'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                                'Position': [0.05, -0.10, 0.06]}}
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
        A program designed to test the data generated from the prediction trained by an individual reinforcement 
        learning algorithm.

        In this case, the target is statically defined and we observe the predicted points of the path. Tthe predicted points 
        are optimized using the least squares method and interpolated using B-Spline.

        More information about the prediction process can be found in the script below:
            ../Static/predict_{CONST_ALGORITHM}.py

        The program to visualize the results in a graph (plot) can be found here:
            ../Static/show_prediction_results.py
    """

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the structure of the main parameters of the environment for the defined robotic arm.
    Env_Structure = PyBullet.Utilities.Get_Environment_Structure(Robot_Str.Name, 0 if CONST_ENV_MODE == 'Default' else 1)

    # Create a static target that was used to predict the path.
    v = np.array([Env_Structure.C.Target.T.p.x + (Env_Structure.C.Target.Size[0]/4.0), 
                  Env_Structure.C.Target.T.p.y + (-1) * (Env_Structure.C.Target.Size[1]/4.0), 
                  Env_Structure.C.Target.T.p.z], dtype=np.float64)
        
    # The name of the path where the file was saved.
    file_path = f'{CONST_PROJECT_FOLDER}/Data/Prediction/Environment_{CONST_ENV_MODE}/{CONST_ALGORITHM}/{Robot_Str.Name}/path_static_target'

    # Read data from the file.
    data = RoLE.Utilities.File_IO.Load(file_path, 'txt', ',')

    # Initialization of a specific class to work with B-Spline curves from 
    # control points.
    S_Cls = B_Spline.B_Spline_Cls(CONST_B_SPLINE['n'], CONST_B_SPLINE['method'], data, 
                                  CONST_B_SPLINE['N'])
    
    # Get optimized control points.
    S_Cls_optimized = S_Cls.Optimize_Control_Points(CONST_NUM_OF_OPT_POINTS)

    # Interpolation of the parametric B-Spline curve from the optimized control points.
    S_optimized = S_Cls_optimized.Interpolate()

    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = PyBullet.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                 CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Home'.
    PyBullet_Robot_Cls.Reset('Home')
    
     # Adding external objects corresponding to a static target that was used to predict the path.
    #   Note:
    #       The size of the sphere corresponds to the threshold distance.
    q_0 = PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all()
    T = HTM_Cls(None, np.float64).Rotation(q_0, 'QUATERNION').Translation(v)
    PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Rand_Viewpoint', T,
                                           None, 0.3, False)
    PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/Sphere/Sphere.urdf', 'T_EE_Rand_Sphere', T,
                                           [0.0, 1.0, 0.0, 0.2], 0.01, False)
    
    # Reset the absolute position of the 'ghost' robot joints to the 'Individual'.
    (_, theta_f) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T, PyBullet_Robot_Cls.Theta, 'Levenberg-Marquardt', Robot_Str, 
                                                                     {'delta_time': 0.2, 'num_of_iteration': 500, 
                                                                      'tolerance': 1e-30})
    PyBullet_Robot_Cls.Reset('Individual', theta_f, True)

    # Calculation of inverse kinematics (IK) using the chosen numerical method.
    theta_0 = PyBullet_Robot_Cls.Theta; theta_arr = []
    for i, data_i in enumerate(S_optimized):
        # Get the homogeneous transformation matrix of the predicted path with index 'i'.
        #   Note:
        #       The orientation is defined from the homogeneous 
        #       transformation matrix of the "Individual" position.
        T_i = HTM_Cls(None, np.float64).Rotation(q_0, 'QUATERNION').Translation(data_i)

        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (_, theta_i) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T_i, theta_0, 'Levenberg-Marquardt', Robot_Str, 
                                                                         CONST_IK_PROPERTIES)
        
        # Store the data.
        theta_arr.append(theta_i)

        # Obtain the last absolute position of the joint.
        theta_0 = theta_i.copy()
    
    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        # If index 'i' is out of range, stop the movement
        if i < len(theta_arr):
            # Set the absolute position of the robot joints.
            _ = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta_arr[i], {'force': 100.0, 't_0': None, 't_1': None})  
            i += 1
        else:
            PyBullet_Robot_Cls.Step()
        
    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()

if __name__ == '__main__':
    sys.exit(main())