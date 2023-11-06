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

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_120_Str
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
    import numpy as np
    theta = Robot_Str.Theta.Home
    
    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Lib.Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Individual'.
    PyBullet_Robot_Cls.Reset('Individual', theta)

    # Get the homogeneous transformation matrix of the robot end-effector.
    T = Kinematics.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]

    # ...
    # 0.349066 -> boundaries in orientation +-
    T_n = T.Translation([0.0, 0.0, -0.1])
    T_n = T_n.Rotation([0.0, 0.0, 0.0], 'ZYX')

    T_obj = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Translation([T.p.x, T.p.y, 0.0])
    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Cube/Cube.urdf', T_obj, [0.0, 1.0, 0.0, 0.25],
                                           0.1, True, False)

    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T_n, None, 
                                           0.5, True, False)
    
    Lib.Gym.Utilities.Add_Wireframe_Cuboid(T_n, [0.1, 0.1, 0.1], [1.0, 0.0, 0.0], 1.0)

    # The physical simulation is in progress.
    while PyBullet_Robot_Cls.is_connected == True:
        PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Motion', {'force': 100.0, 't_0': 0.0, 't_1': 3.0})
        #x = PyBullet_Robot_Cls.Set_TCP_Position(T_n, 'Reset')
        #print(np.round(np.rad2deg(PyBullet_Robot_Cls.Theta), 5))
        #pass

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
