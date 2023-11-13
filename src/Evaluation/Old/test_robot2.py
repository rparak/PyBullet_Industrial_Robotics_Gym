# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Time (Time access and conversions)
import time
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Gym/Core
import Lib.Gym.Core

# ...
import Lib.Transformation.Core as Transformation
# ...
import numpy as np

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': None,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        A program to test the functionality of the designed class.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Lib.Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Home'.
    PyBullet_Robot_Cls.Reset('Home')

    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Cube/Cube.urdf', 'Cube_1', PyBullet_Robot_Cls.T_EE.Translation([0.0, 0.0, -0.1]), [0.0, 1.0, 0.0, 1.0], 0.05, True, False)
    
    vertices = PyBullet_Robot_Cls.Get_Configuration_Space_Vertices('Search')

    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        # ...
        #T_EE_rand = PyBullet_Robot_Cls.Generate_Random_T_EE('Target', True)
        # ...
        #time.sleep(1.0)
        # ...
        #PyBullet_Robot_Cls.Step()

        T = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation(vertices[i])
        _ = PyBullet_Robot_Cls.Set_TCP_Position(T, 'Motion', {'force': 100.0, 't_0': 0.0, 't_1': 2.0})

        time.sleep(1.0)

        if i < vertices.shape[0] - 1:
            i += 1
        else:
            i = 0


    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
