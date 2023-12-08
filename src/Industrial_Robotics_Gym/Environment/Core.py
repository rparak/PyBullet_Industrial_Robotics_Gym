# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
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
    def __init__(self, mode='Default', Robot_Str=Parameters.Universal_Robots_UR3_Str, action_step_factor=0.05, distance_threshold=0.05):
        super(Industrial_Robotics_Gym_Env_Cls, self).__init__()

        # ...
        self.__distance_threshold = np.float32(distance_threshold)
        self.__action_step_factor = np.float32(action_step_factor)

        # ...
        self.__Set_Env_Parameters(mode, Robot_Str)

        # ...
        observation, _ = self.reset()

        # ...
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3, ), dtype=np.float32)
        self.observation_space = gym.spaces.Dict({'observation': gym.spaces.Box(-1.0, 1.0, shape=observation['observation'].shape, dtype=np.float32),
                                                  'achieved_goal': gym.spaces.Box(-1.0, 1.0, shape=observation['achieved_goal'].shape, dtype=np.float32),
                                                  'desired_goal': gym.spaces.Box(-1.0, 1.0, shape=observation['desired_goal'].shape, dtype=np.float32)})

    def __Set_Env_Parameters(self, mode, Robot_Str):
        # Numerical IK Parameters.
        #   The properties of the inverse kinematics solver.
        self.__ik_properties = {'delta_time': 0.20, 'num_of_iteration': 100, 
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
        self.__p_0 = self.__T_0.p.all().astype(np.float32); self.__q_0 = self.__T_0.Get_Rotation('QUATERNION').all().astype(np.float32)

        # ...
        vertices_C_target = self.__PyBullet_Robot_Cls.Get_Configuration_Space_Vertices('Target')

        # Get the minimum and maximum X, Y, Z values of the input vertices.
        (min_vec3, max_vec3) = Get_Min_Max(vertices_C_target)
        self.__min_vec3 = min_vec3.astype(np.float32)
        self.__max_vec3 = max_vec3.astype(np.float32)

        # ...
        self.__PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Rand_Viewpoint', HTM_Cls(None, np.float32),
                                                      None, 0.3, False)
        self.__PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/Sphere/Sphere.urdf', 'T_EE_Rand_Sphere', HTM_Cls(None, np.float32),
                                                      [0.0, 1.0, 0.0, 0.2], self.__distance_threshold, False)

    @staticmethod
    def __Euclidean_Norm(x: tp.List[float]) -> tp.List[float]:
        """
            ...
        """
        return np.linalg.norm(x, axis=-1)

    def compute_reward(self, p: tp.List[float], p_1: tp.List[float], info: tp.Dict[str, tp.Any] = {}) -> tp.List[float]:
        return -self.__Euclidean_Norm(p - p_1).astype(np.float32)
    
    def is_success(self, p, p_1):
        return np.array(self.__Euclidean_Norm(p - p_1) < self.__distance_threshold, dtype=bool)
    
    def step(self, action):
        # ...
        action = action.copy()
        action = np.clip(action, self.action_space.low, self.action_space.high)
    
        # ...
        self.__p = self.__PyBullet_Robot_Cls.T_EE.p.all().copy().astype(np.float32) + action * self.__action_step_factor
        
        # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
        (successful, theta) = self.__PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(HTM_Cls(None, np.float32).Rotation(self.__q_0, 'QUATERNION').Translation(self.__p), 
                                                                                        self.__ik_properties, False)
        
        # ...
        self.__PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, 100.0, 0.0, 0.1)

        # ...
        truncated = not successful

        # ...
        terminated = bool(self.is_success(self.__p, self.__p_1))

        # ...
        info = {'is_success': terminated}

        # ...
        reward = float(self.compute_reward(self.__p, self.__p_1, info))

        # ...
        observation = np.concatenate([self.__p, self.__PyBullet_Robot_Cls.T_EE_v[0:3]], dtype=np.float32)

        return ({'observation': observation,
                 'achieved_goal': self.__p.astype(np.float32),
                 'desired_goal': self.__p_1.astype(np.float32)}, 
                reward,
                terminated,
                truncated,
                info)
    
    def reset(self, seed=None, options=None):
        # ...
        super().reset(seed=seed, options=options)
        self.np_random, seed = gym.utils.seeding.np_random(seed)

        # ...
        self.__p_1 = self.np_random.uniform(self.__min_vec3, self.__max_vec3).astype(np.float32)

        # Reset the absolute position of the auxiliary robotic structure, which is represented 
        # as a 'ghost', to 'Home'.
        self.__PyBullet_Robot_Cls.Reset(mode='Home', enable_ghost=False)

        # ...
        T_rand = HTM_Cls(None, np.float32).Rotation(self.__q_0, 'QUATERNION').Translation(self.__p_1)
        self.__PyBullet_Robot_Cls.Transformation_External_Object('T_EE_Rand_Viewpoint', T_rand, False)
        self.__PyBullet_Robot_Cls.Transformation_External_Object('T_EE_Rand_Sphere', T_rand, False)

        # ...
        self.__p = self.__p_0.copy().astype(np.float32)

        # ...
        observation = np.concatenate([self.__p, self.__PyBullet_Robot_Cls.T_EE_v[0:3]], dtype=np.float32)

        return ({'observation': observation,
                 'achieved_goal': self.__p,
                 'desired_goal': self.__p_1}, 
                {'is_success': self.is_success(self.__p_0, self.__p_1)})

    def close(self):
        # Disconnect the created environment from a physical server.
        self.__PyBullet_Robot_Cls.Disconnect()