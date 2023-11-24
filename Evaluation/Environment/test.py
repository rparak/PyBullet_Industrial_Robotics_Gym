# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   Gym
#       ../Lib/Gym/Core
import Gym.Core

import RoLE.Transformation.Core as Transformation
import numpy as np
import RoLE.Transformation.Utilities.Mathematics as Mathematics
"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_R_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf', 'Env_ID': 1,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        A program that tests whether the environment has been successfully generated.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                            CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Home'.
    PyBullet_Robot_Cls.Reset('Home') 

    #print(PyBullet_Robot_Cls.T_EE.p)
    T = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation([0.385, 0.185, 0.220])

    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    #PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Viewpoint', T, None, 0.3, False)
    
    #T_rand = PyBullet_Robot_Cls.Generate_Random_T_EE('Target', True)
    #T_new = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation([T_rand.p.x, T_rand.p.y, T_rand.p.z - 0.01])

    # Get the vertices of the selected configuration space.
    C_vertices = PyBullet_Robot_Cls.Get_Configuration_Space_Vertices('Search')
    T_vertex = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation(C_vertices[6])

    # The physical simulation is in progress.
    while PyBullet_Robot_Cls.is_connected == True:
        _ = PyBullet_Robot_Cls.Set_TCP_Position(T_vertex, 'Reset', {'delta_time': 0.1, 'num_of_iteration': 500, 'tolerance': 1e-30}, False)
        #e = Mathematics.Euclidean_Norm((PyBullet_Robot_Cls.T_EE.p - T_rand.p).all())
        #print(e)
        #_ = PyBullet_Robot_Cls.Set_TCP_Position(T, 'Reset', {'delta_time': 0.1, 'num_of_iteration': 500, 'tolerance': 1e-30}, False)
        #print(np.round(np.rad2deg(PyBullet_Robot_Cls.Theta), 5).tolist())
        #print(np.round(PyBullet_Robot_Cls.Theta, 5).tolist())
        #PyBullet_Robot_Cls.Step()


    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
