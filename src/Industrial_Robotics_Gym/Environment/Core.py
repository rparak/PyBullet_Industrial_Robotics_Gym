# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# OS (Operating system interfaces)
import os
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
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
        self.observation_space = gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float64)

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

    def __Calculate_Reward(self, T, T_i):
        d = Mathematics.Euclidean_Norm((T.p - T_i.p).all())
        if self.__reward_type == 'Sparse':
            return -d.astype(np.float64)
        else:
            return -np.array(d > self.__distance_threshold, dtype=np.float64)
        
    def __Is_Successful(self, T, T_i):
        return np.array(Mathematics.Euclidean_Norm((T.p - T_i.p).all()) < self.__distance_threshold, dtype=bool)

    def step(self, action):
        return None
    
    def reset(self):
        return None

    def close(self):
        # Disconnect the created environment from a physical server.
        #PyBullet_Robot_Cls.Disconnect()
        pass