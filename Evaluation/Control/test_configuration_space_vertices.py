# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Numpy (Arline computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Time (Time access and conversions)
import time
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   Gym
#       ../Lib/Gym/Core
import Gym.Core

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
CONST_VISIBILITY_GHOST = True
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': None, 'Env_ID': 1,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}
# Type of the configuration space.
#   Note:
#       'Search' or 'Target'
CONST_C_TYPE = 'Search'
# The name of the mode to be used to perform the transformation.
#   Note:
#       mode = 'Reset' or 'Motion'
CONST_CTRL_MODE = 'Motion'

def main():
    """
    Description:
        A program to test the search/target (configuration) space by checking the vertices within a defined cuboid area.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                            CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Home'.
    PyBullet_Robot_Cls.Reset('Home')

    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Viewpoint', PyBullet_Robot_Cls.T_EE, None, 
                                           0.3, False)
    
    # Get the vertices of the selected configuration space.
    C_vertices = PyBullet_Robot_Cls.Get_Configuration_Space_Vertices(CONST_C_TYPE)

    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        # Get the homogeneous transformation matrix of the vertex with index 'i'.
        #   Note:
        #       The orientation of the vertex is defined by the homogeneous transformation 
        #       matrix of the 'Home' position.
        T_vertex = HTM_Cls(None, np.float64).Rotation(PyBullet_Robot_Cls.T_EE.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation(C_vertices[i])

        # Add a viewpoint with the correct transformation to the vertex with index 'i'.
        PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'Viewpoint_i', T_vertex, None, 
                                               0.3, False)

        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (successful, theta) = PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(T_vertex, CONST_IK_PROPERTIES, CONST_VISIBILITY_GHOST)

        # Set the absolute position of the robot joints.
        in_position = False
        if successful == True:
            in_position = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, 100.0, 0.0, 2.0)

        if in_position == False or successful == False:
            print('[WARNING] There is an issue during the execution of the TCP (tool center point) target.')
            print(f'[WARNING] >> p = {T_vertex.p.all()}')
            print(f'[WARNING] >> Quaternion = {T_vertex.Get_Rotation("QUATERNION").all()}')

        # Pause for a defined time.
        time.sleep(1.0)

        # Reset the absolute position of the robot joints to the 'Home'.
        PyBullet_Robot_Cls.Reset('Home')

        # If index 'i' is out of range, reset the counter.
        i = i + 1 if i < C_vertices.shape[0] - 1 else 0

        # Remove the specified model from the PyBullet environment.
        PyBullet_Robot_Cls.Remove_External_Object('Viewpoint_i')

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
