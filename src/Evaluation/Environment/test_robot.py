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

    print(PyBullet_Robot_Cls.T_EE.p.all())

    # The physical simulation is in progress.
    while PyBullet_Robot_Cls.is_connected == True:
        # ...
        T_EE_rand = PyBullet_Robot_Cls.Generate_Random_T_EE('Target')

        # Add a viewpoint (+ sphere) with the correct transformation to the end-effector of the structure.
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Sphere/Sphere.urdf', T_EE_rand, [0.0, 1.0, 0.0, 0.25], 
                                               0.015, True, False)
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T_EE_rand, None, 
                                               0.3, True, False)

        # ...
        time.sleep(1.0)

        # ...
        PyBullet_Robot_Cls.Remove_All_External_Objects()

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
