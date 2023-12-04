# System (Default)
import sys
#   Add access if it is not in the system path.
if '..' not in sys.path:
    sys.path.append('..')
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium.envs.registration
# Custom Lib.:
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters

"""
Description:
    ...

    Notes:
        kwargs []: 
        max_episode_steps [int]: The maximum number of steps that an episode can consist of.
"""

gymnasium.envs.registration.register(
    id='IndustrialRoboticsReach-v0',
    entry_point='Industrial_Robotics_Gym.Environment.Core:Industrial_Robotics_Gym_Env_Cls',
    kwargs={'mode': 'Default',
            'Robot_Str': Parameters.Universal_Robots_UR3_Str,
            'reward_type': 'Dense', 
            'distance_threshold': 0.01},
    max_episode_steps=100,
)