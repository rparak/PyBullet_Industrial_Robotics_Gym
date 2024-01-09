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
#   ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Utilities/File_IO
import RoLE.Utilities.File_IO
#   ../RoLE/Interpolation/B_Spline/Core
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
CONST_IK_PROPERTIES = {'delta_time': 0.1, 'num_of_iteration': 500, 
                       'tolerance': 1e-30}
# Visibility of the target position as the 'ghost' of the robotic model.
CONST_VISIBILITY_GHOST = False
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': True, 'fps': 100, 
                                 'External_Base': None, 'Env_ID': 0,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}
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
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

def main():
    """
    Description:
        ....
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
    
    # Initialization of a specific class to work with B-Spline curves.
    S_Cls = B_Spline.B_Spline_Cls(CONST_B_SPLINE['n'], CONST_B_SPLINE['method'], data, 
                                  CONST_B_SPLINE['N'])
    
    # Interpolation of parametric B-Spline curve.
    S = S_Cls.Interpolate()

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
    
    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        # Get the homogeneous transformation matrix of the predicted path with index 'i'.
        #   Note:
        #       The orientation is defined from the homogeneous 
        #       transformation matrix of the "Individual" position.
        T_i = HTM_Cls(None, np.float64).Rotation(q_0, 'QUATERNION').Translation(data[i, :])

        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (successful, theta) = PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(T_i, CONST_IK_PROPERTIES, CONST_VISIBILITY_GHOST)

        # Set the absolute position of the robot joints.
        in_position = False
        if successful == True:
            in_position = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, {'force': 100.0, 't_0': 0.0, 't_1': 0.1})
        
        if in_position == False or successful == False:
            print('[WARNING] There is an issue during the execution of the TCP (tool center point) target.')
            print(f'[WARNING] >> p = {T_i.p.all()}')
            print(f'[WARNING] >> Quaternion = {T_i.Get_Rotation("QUATERNION").all()}')
            break

        # If index 'i' is out of range, break the cycle.
        if i < data[:, 0].size - 1:
            i += 1
        else:
            break
        
    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()

if __name__ == '__main__':
    sys.exit(main())