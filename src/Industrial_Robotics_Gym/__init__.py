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
    Initialization of constants.
"""
# The name of the environment mode.
#   'Default': 
#       A mode called "Default" that demonstrates an environment without a collision object.
#   'Safe': 
#       A mode called "Safe" that demonstrates an environment with a collision object.
CONST_ENV_MODE = ['Default', 'Safe']
# The name of a particular robotic structure.
CONST_ROBOT_NAME = ['Ur3', 'AbbIrb120', 'AbbIrb120L', 'AbbIrb14000L', 'AbbIrb14000R', 'EpsonLs3']
# Set the structure of the main parameters of the robot.
CONST_ROBOT_TYPE = [Parameters.Universal_Robots_UR3_Str, Parameters.ABB_IRB_120_Str, Parameters.ABB_IRB_120_L_Ax_Str,
                    Parameters.ABB_IRB_14000_L_Str, Parameters.ABB_IRB_14000_R_Str, Parameters.EPSON_LS3_B401S_Str]

# The part that registers the gym environment with an additional parameters.
for _, env_mode_i in enumerate(CONST_ENV_MODE):
    for _, (r_name_i, r_str_i) in enumerate(zip(CONST_ROBOT_NAME, CONST_ROBOT_TYPE)):
        gymnasium.envs.registration.register(
            id=f'{r_name_i}-{env_mode_i}-Reach-v0',
            entry_point='Industrial_Robotics_Gym.Environment.Core:Industrial_Robotics_Gym_Env_Cls',
            kwargs={'mode': env_mode_i,
                    'Robot_Str': r_str_i,
                    'action_step_factor': 0.04,
                    'distance_threshold': 0.01,
                    'T': None},
            max_episode_steps=100)