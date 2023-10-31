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
#   ../Lib/PyBullet/Core
import Lib.PyBullet.Core
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.ABB_IRB_14000_R_Str
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Template_Industrial_Robotics')[0] + 'PyBullet_Template_Industrial_Robotics'
# The properties of the PyBullet environment.
#   Note:
#      ABB_IRB_14000_{L, R}_Str:
#       'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
CONST_PYBULLET_ENV_PROPERTIES = {'Enable_GUI': 0, 'fps': 100, 
                                 'External_Base': f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf',
                                 'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                            'Position': [0.05, -0.10, 0.06]}}

def main():
    """
    Description:
        A program that demonstrates simple trajectory generation in the PyBullet environment using a designed class.
    """
    
    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    # Obtain the desired absolute position of the robot joints.
    theta = Robot_Str.Theta.Home
    
    # Initialization of the class to work with a robotic arm object in a PyBullet environment.
    PyBullet_Robot_Cls = Lib.PyBullet.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                     CONST_PYBULLET_ENV_PROPERTIES)
    
    # Reset the absolute position of the robot joints to the 'Zero'.
    PyBullet_Robot_Cls.Reset('Zero')

    # The physical simulation is in progress.
    while PyBullet_Robot_Cls.is_connected == True:
        # Get the homogeneous transformation matrix of the robot end-effector.
        T = Kinematics.Forward_Kinematics(theta, 'Fast', Robot_Str)[1]

        # Add a viewpoint (+ sphere) with the correct transformation to the end-effector of the structure.
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Primitives/Sphere/Sphere.urdf', T, [0.0, 1.0, 0.0, 0.25], 
                                               0.025, True, False)
        PyBullet_Robot_Cls.Add_External_Object('/../../../URDFs/Viewpoint/Viewpoint.urdf', T, None, 
                                               0.5, True, False)
            
        # Set the absolute position of the robot joints.
        in_position = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, 100.0, 0.0, 5.0)
        
        if in_position == True:
            # Change the desired absolute position of the robot joint.
            theta = Robot_Str.Theta.Home if (theta == Robot_Str.Theta.Zero).all() else Robot_Str.Theta.Zero

            # Remove all models with from the PyBullet environment.
            PyBullet_Robot_Cls.Remove_All_External_Objects()
            in_position = False

    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()
    
if __name__ == '__main__':
    main()
