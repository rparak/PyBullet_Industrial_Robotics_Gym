# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.: 
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core as Kinematics
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   PyBullet
#       ../PyBullet/Core
import PyBullet.Core

"""
Description:
    Initialization of constants.
"""
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

class Industrial_Robotics_Gym_Env_Cls(gym.Env):
    def __init__(self, mode='Default', Robot_Str=Parameters.Universal_Robots_UR3_Str, reward_type='Dense', distance_threshold=0.01):
        super(Industrial_Robotics_Gym_Env_Cls, self).__init__()

        # ...
        self.__reward_type = reward_type
        self.__distance_threshold = distance_threshold

        # ...
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3, ), dtype=np.float64)
        self.observation_space = gym.spaces.Dict({'p_0': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float64),
                                                  'p_1': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float64),
                                                  'p': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float64)})

        # ...
        self.__T_1 = None; self.__T = None

        # Numerical IK Parameters.
        #   The properties of the inverse kinematics solver.
        self.__ik_properties = {'delta_time': 0.1, 'num_of_iteration': 500, 
                                'tolerance': 1e-30}

        # ...
        if 'ABB_IRB_14000' in Robot_Str.Name:
            external_base = f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
        else:
            external_base = None

        # The properties of the PyBullet environment. 
        pybullet_env_properties = {'Enable_GUI': 0, 'fps': 100, 
                                   'External_Base': external_base, 'Env_ID': 0 if mode == 'Default' else 1,
                                   'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                              'Position': [0.05, -0.10, 0.06]}}
        
        # Initialization of the class to work with a robotic arm object in a PyBullet environment.
        self.__PyBullet_Robot_Cls = PyBullet.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                            pybullet_env_properties)
        
        # Get the homogeneous transformation matrix of the robot end-effector in the 'Home' position.
        self.__T_0 = Kinematics.Forward_Kinematics(Robot_Str.Theta.Home, 'Fast', Robot_Str)[1]
        #   Get the translational and rotational part from the transformation matrix.
        self.__p_0 = self.__T_0.p.all(); self.__q_0 = self.__T_0.Get_Rotation('QUATERNION').all()

    def __Calculate_Reward(self, T, T_1):
        d = Mathematics.Euclidean_Norm((T_1.p - T.p).all())
        if self.__reward_type == 'Sparse':
            return -np.array(d > self.__distance_threshold, dtype=np.float64)
        else:
            return -d.astype(np.float64)
        
    def __Is_Successful(self, T, T_1):
        return np.array(Mathematics.Euclidean_Norm((T_1.p - T.p).all()) < self.__distance_threshold, dtype=bool)

    def step(self, action):
        # ...
        p = np.array(self.__T.p.all(), dtype=np.float64) + action * self.__distance_threshold
        #   ...
        self.__T = HTM_Cls(None, np.float64).Rotation(self.__q_0, 'QUATERNION').Translation(p)
        
        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (successful, _) = self.__PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(self.__T, self.__ik_properties, True)

        # ...
        warning = not successful

        # ...
        reward = self.__Calculate_Reward(self.__T, self.__T_1)

        # ...
        successful = self.__Is_Successful(self.__T, self.__T_1)

        return ({'p_0': np.array(self.__T_0.p.all(), dtype=np.float64),
                 'p_1': np.array(self.__T_1.p.all(), dtype=np.float64),
                 'p': np.array(self.__T.p.all(), dtype=np.float64)}, 
                 reward,
                 successful, 
                 warning,
                {'is_successful': self.__Is_Successful(self.__T, self.__T_1)})
    
    def  reset(self, seed=None, options=None):
        # ...
        super().reset(seed=seed, options=options)
        self.np_random, seed = gym.utils.seeding.np_random(seed)

        # Generate the homogeneous transformation matrix of a random end-effector position 
        # within the defined configuration space.
        self.__T_1 = self.__PyBullet_Robot_Cls.Generate_Random_T_EE('Target', True)

        # Reset the absolute position of the robot joints to the 'Home'.
        self.__PyBullet_Robot_Cls.Reset('Home')

        # ...
        self.__T = HTM_Cls(None, np.float64).Rotation(self.__q_0, 'QUATERNION').Translation(self.__p_0)

        return ({'p_0': np.array(self.__T_0.p.all(), dtype=np.float64),
                 'p_1': np.array(self.__T_1.p.all(), dtype=np.float64),
                 'p': np.array(self.__T.p.all(), dtype=np.float64)}, 
                {'is_successful': self.__Is_Successful(self.__T, self.__T_1)})

    def close(self):
        # Disconnect the created environment from a physical server.
        self.__PyBullet_Robot_Cls.Disconnect()