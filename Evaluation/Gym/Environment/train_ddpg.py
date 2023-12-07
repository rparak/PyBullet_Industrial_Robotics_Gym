# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../../' + 'src' not in sys.path:
    sys.path.append('../../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Stable-Baselines3 (A set of implementations of reinforcement learning algorithms in PyTorch) [pip3 install stable-baselines3]
import stable_baselines3 
import stable_baselines3.common.noise
import stable_baselines3.common.logger
import stable_baselines3.common.monitor
import stable_baselines3.common.vec_env
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   Industrial_Robotics_Gym
#       ../Industrial_Robotics_Gym
import Industrial_Robotics_Gym
#       ../Industrial_Robotics_Gym/Utilities
import Industrial_Robotics_Gym.Utilities

"""
Description:
    Initialization of constants.
"""
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = Parameters.Universal_Robots_UR3_Str
# ...
CONST_MODE = 'Default'

def main():

    # Initialization of the structure of the main parameters of the robot.
    Robot_Str = CONST_ROBOT_TYPE

    tmp_path = "./DDPG_results"
    # set up logger
    new_logger = stable_baselines3.common.logger.configure(tmp_path, ['stdout', 'csv'])

    # ...
    env = stable_baselines3.common.monitor.Monitor(gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID(Robot_Str.Name, CONST_MODE)), tmp_path)
    env = stable_baselines3.common.vec_env.DummyVecEnv([lambda: env])

    # ...
    n_actions = env.action_space.shape[-1]
    action_noise = stable_baselines3.common.noise.NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1*np.ones(n_actions))

    t_0 = time.time()
    model = stable_baselines3.DDPG(policy="MultiInputPolicy", env=env, gamma=0.95, learning_rate=0.001, action_noise=action_noise, device='cuda', 
                                   batch_size=256, policy_kwargs=dict(net_arch=[256, 256, 256]), verbose=1)
    model.set_logger(new_logger)
    model.learn(total_timesteps=100000, log_interval=10)
    model.save('model')

    print(time.time() - t_0)

if __name__ == '__main__':
    main()
