# System (Default)
import sys
#   Add access if it is not in the system path.
if '../' + 'src' not in sys.path:
    sys.path.append('../..')
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Gym/Core
import Lib.Gym.Core
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics

# ...
import Lib.Transformation.Core as Transformation
# ...
import Lib.Gym.Utilities
# ...
import numpy as np
import time

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_L_Str
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

    # Obtain the desired absolute position of the robot joints.
    theta = Robot_Str.Theta.Home
    
    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Lib.Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Individual'.
    PyBullet_Robot_Cls.Reset('Individual', theta)

    # Get the homogeneous transformation matrix of the robot end-effector.
    T = Kinematics.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]
    #print(T.p)
    # ...
    # 0.349066 -> boundaries in orientation +-
    #T_n = T.Translation([0.0, 0.0, 0.0])
    #T_n = T_n.Rotation([0.0, 0.0, 0.0], 'ZYX')


    #print(T.p.x + 0.05, T.p.y, 0.05)

    """
    T_obj = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Translation([T.p.x + 0.05, T.p.y, 0.05])
    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Cube/Cube.urdf', T_obj, [0.0, 1.0, 0.0, 0.25],
                                           0.1, True, False)
    """
    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T, None, 0.5, True, False)
    
    C = Lib.Gym.Utilities.Get_Configuration_Space(Robot_Str.Name)
    _ = Lib.Gym.Utilities.Add_Wireframe_Cuboid(C.Search.T, C.Search.Size, 
                                               C.Search.Color, 1.0)
    vertices = Lib.Gym.Utilities.Add_Wireframe_Cuboid(C.Target.T, C.Target.Size, 
                                                      C.Target.Color, 1.0)
    
    #print(vertices)

    #T_n = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(T.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation([0.35, -0.15, 0.225])
    #PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T_n, None, 0.5, True, False)

    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        T_n = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(T.Get_Rotation('QUATERNION').all(), 'QUATERNION').Translation(vertices[i])
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T_n, None, 0.5, True, False)
        x = PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Motion', {'force': 100.0, 't_0': 0.0, 't_1': 2.0})
        #x = PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Reset')
        time.sleep(2.0)

        if i < vertices.shape[0] - 1:
            i += 1
        else:
            i = 0

        PyBullet_Robot_Cls.Remove_All_External_Objects()
        PyBullet_Robot_Cls.Reset('Individual', Robot_Str.Theta.Home)
        #x = PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Motion', {'force': 100.0, 't_0': 0.0, 't_1': 3.0})
        #x = PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Reset')
        #print(np.round(np.rad2deg(PyBullet_Robot_Cls.Theta), 5).tolist())
        pass

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
