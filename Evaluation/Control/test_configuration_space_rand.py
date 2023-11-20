# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# OS (Operating system interfaces)
import os
# Time (Time access and conversions)
import time
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
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
CONST_VISIBILITY_GHOST = False
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': None, 'Env_ID': 0,
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}
# Type of the configuration space.
#   Note:
#       'Search' or 'Target'
CONST_C_TYPE = 'Target'
# The name of the mode to be used to perform the transformation.
#   Note:
#       mode = 'Reset' or 'Motion'
CONST_CTRL_MODE = 'Reset'

def main():
    """
    Description:
        A program to test the search/target (configuration) space by generating a random point within a defined cuboid area.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Gym.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                            CONST_PYBULLET_ENV_PROPERTIES)

    # Reset the absolute position of the robot joints to the 'Home'.
    PyBullet_Robot_Cls.Reset('Home')

    # Add a viewpoint with the correct transformation to the end-effector of the structure.
    PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/Sphere/Sphere.urdf', 'T_EE_Sphere', PyBullet_Robot_Cls.T_EE, 
                                           [0.0, 1.0, 0.0, 0.25], 0.015, False)
    
    # The physical simulation is in progress.
    while PyBullet_Robot_Cls.is_connected == True:
        # Generate the homogeneous transformation matrix of a random end-effector position 
        # within the defined configuration space.
        T_rand = PyBullet_Robot_Cls.Generate_Random_T_EE(CONST_C_TYPE, True)

        # Set the TCP (tool center point) of the robot end-effector.
        in_position = PyBullet_Robot_Cls.Set_TCP_Position(T_rand, CONST_CTRL_MODE, CONST_IK_PROPERTIES, CONST_VISIBILITY_GHOST,
                                                          {'force': 100.0, 't_0': 0.0, 't_1': 2.0})

        if in_position == False:
            print('[WARNING] There is an issue during the execution of the TCP (tool center point) target.')
            print(f'[WARNING] >> p = {T_rand.p.all()}')
            print(f'[WARNING] >> Quaternions = {T_rand.Get_Rotation("QUATERNION").all()}')
            break

        # Pause for a defined time.
        time.sleep(2.0)

        # Reset the absolute position of the robot joints to the 'Home'.
        PyBullet_Robot_Cls.Reset('Home')

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()