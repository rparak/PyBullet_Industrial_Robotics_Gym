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
#       ../RoLE/Collider/Utilities
from RoLE.Collider.Utilities import Get_Min_Max
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
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3, ), dtype=np.float32)
        self.observation_space = gym.spaces.Dict({'observation': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float32),
                                                  'achieved_goal': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float32),
                                                  'desired_goal': gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float32)})

        # ...
        self.__p_1 = None; self.__p = None

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
        
        # Reset the absolute position of the robot joints to the 'Home'.
        self.__PyBullet_Robot_Cls.Reset(mode='Home', enable_ghost=False)

        # Get the homogeneous transformation matrix of the robot end-effector in the 'Home' position.
        self.__T_0 = Kinematics.Forward_Kinematics(Robot_Str.Theta.Home, 'Fast', Robot_Str)[1]
        #   Get the translational and rotational part from the transformation matrix.
        self.__p_0 = self.__T_0.p.all(); self.__q_0 = self.__T_0.Get_Rotation('QUATERNION').all()

        # ...
        vertices_C_target = self.__PyBullet_Robot_Cls.Get_Configuration_Space_Vertices('Target')

        # Get the minimum and maximum X, Y, Z values of the input vertices.
        (self.__min_vec3, self.__max_vec3) = Get_Min_Max(vertices_C_target)

    def compute_reward(self, p, p_1, info = {}):
        d = Mathematics.Euclidean_Norm(p - p_1)
        if self.__reward_type == 'Sparse':
            return -np.array(d > self.__distance_threshold, dtype=np.float32)
        else:
            return -d.astype(np.float32)
        
    def is_success(self, p, p_1):
        return (Mathematics.Euclidean_Norm(p - p_1) < self.__distance_threshold).astype(dtype=bool)

    def step(self, action):
        # ...
        self.__p += action * np.float32(0.05)
        
        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (successful, _) = self.__PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(HTM_Cls(None, np.float32).Rotation(self.__q_0, 'QUATERNION').Translation(self.__p), 
                                                                                    self.__ik_properties, True)

        # ...
        truncated = not successful

        # ...
        reward = self.compute_reward(self.__p, self.__p_1)

        # ...
        terminated = self.is_success(self.__p, self.__p_1)

        return ({'observation': self.__p_0.astype('float32'),
                 'achieved_goal': self.__p.astype('float32'),
                 'desired_goal': self.__p_1.astype('float32')}, 
                 reward,
                 terminated,
                 truncated,
                {'is_success': self.is_success(self.__p, self.__p_1)})
    
    def  reset(self, seed=None, options=None):
        # ...
        super().reset(seed=seed, options=options)
        self.np_random, seed = gym.utils.seeding.np_random(seed)

        # ...
        self.__p_1 = self.np_random.uniform(self.__min_vec3, self.__max_vec3)

        # Reset the absolute position of the auxiliary robotic structure, which is represented 
        # as a 'ghost', to 'Home'.
        self.__PyBullet_Robot_Cls.Reset(mode='Home', enable_ghost=True)

        # ...
        self.__p = self.__p_0.copy()

        return ({'observation': self.__p_0.astype('float32'),
                 'achieved_goal': self.__p.astype('float32'),
                 'desired_goal': self.__p_1.astype('float32')}, 
                {'is_success': self.is_success(self.__p_0, self.__p_1)})

    def close(self):
        # Disconnect the created environment from a physical server.
        self.__PyBullet_Robot_Cls.Disconnect()