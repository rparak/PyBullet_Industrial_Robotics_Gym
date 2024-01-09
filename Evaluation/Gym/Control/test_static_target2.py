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
#       ../RoLE/Trajectory/Core
import RoLE.Trajectory.Core
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core
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

    # ...
    s = []
    for _, data_i in enumerate(data.T):
        # Initialization of multi-segment constraints for trajectory generation.
        #  1\ Input control points (waypoints) to be used for trajectory generation.
        P_i = data_i.copy()
        #  2\ Trajectory duration between control points.
        delta_T = np.array((P_i.size - 1) * [2.5], dtype=np.float32)
        #  3\ Duration of the blend phase.
        t_blend = np.array(P_i.size * [0.5], dtype=np.float32)

        # Initialization of the class to generate multi-segment trajectory.
        MST_Cls = RoLE.Trajectory.Core.Multi_Segment_Cls('Trapezoidal', delta_time=0.1)
    
        # Generation of position multi-segment trajectories from input parameters.
        (s_i, _, _, _, _) = MST_Cls.Generate(P_i, delta_T, t_blend)

        # ...
        s.append(s_i)
    
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
    
    # Calculation of inverse kinematics (IK) using the chosen numerical method.
    theta_0 = PyBullet_Robot_Cls.Theta; theta_arr = []
    for i, S_i in enumerate(np.array(s, dtype=np.float32).T):
        # Get the homogeneous transformation matrix of the predicted path with index 'i'.
        #   Note:
        #       The orientation is defined from the homogeneous 
        #       transformation matrix of the "Individual" position.
        T_i = HTM_Cls(None, np.float64).Rotation(q_0, 'QUATERNION').Translation(S_i)

        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (info, theta_i) = RoLE.Kinematics.Core.Inverse_Kinematics_Numerical(T_i, theta_0, 'Levenberg-Marquardt', Robot_Str, 
                                                                            CONST_IK_PROPERTIES)

        # Check whether the inverse kinematics (IK) has a solution or not.
        #   Conditions:
        #       1\ IK solution within limits.
        #       2\ Collision-free.
        #       3\ No singularities.
        if info['successful'] == True:
            # Check whether a part of the robotic structure collides with external objects.
            (is_external_collision, _) = RoLE.Kinematics.Core.General.Is_External_Collision(theta_i, Robot_Str)

            if info['is_self_collision'] == False and info['is_close_singularity'] == False \
                and is_external_collision == False:
                    successful = True
            else:
                successful = False
        else:
            successful = False

        if successful == False:
            print(f'[WARNING] The calculation of IK stopped in iteration {i}.')
            print(f'[WARNING] >> is_self_collision = {info["is_self_collision"]}')
            print(f'[WARNING] >> is_close_singularity = {info["is_close_singularity"]}')
            break

        theta_arr.append(theta_i)

        # Obtain the last absolute position of the joint.
        theta_0 = theta_i.copy()

    # The physical simulation is in progress.
    i = 0
    while PyBullet_Robot_Cls.is_connected == True:
        # Set the absolute position of the robot joints.
        _ = PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta_arr[i], {'force': 100.0, 't_0': 0.0, 't_1': 0.1})  

        # If index 'i' is out of range, break the cycle.
        if i < len(theta_arr) - 1:
            i += 1
        else:
            #break
            pass
        
    # Disconnect the created environment from a physical server.
    PyBullet_Robot_Cls.Disconnect()

if __name__ == '__main__':
    sys.exit(main())