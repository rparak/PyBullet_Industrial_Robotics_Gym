# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics

class Industrial_Robotics_Gym_Env_Cls(gym.Env):
    def __init__(self, reward_type='Dense', robot_type = 'Universal_Robots_UR3', distance_threshold = 0.05):
        super(Industrial_Robotics_Gym_Env_Cls, self).__init__()

        # ...
        self.__reward_type = reward_type
        self.__distance_threshold = distance_threshold

        # ...
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3, ), dtype=np.float64)
        self.observation_space = gym.spaces.Box(-10.0, 10.0, shape=(3, ), dtype=np.float64)

    def __Calculate_Reward(self, p, p_i):
        d = Mathematics.Euclidean_Norm(p - p_i)
        if self.__reward_type == 'Sparse':
            return -d.astype(np.float64)
        else:
            return -np.array(d > self.__distance_threshold, dtype=np.float64)
        
    def __Is_Successful(self, p, p_i):
        return np.array(Mathematics.Euclidean_Norm(p - p_i) < self.__distance_threshold, dtype=bool)

    def step(self, action):
        return None
    
    def reset(self):
        return None

    def close (self):
        pass