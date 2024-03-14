"""
## =========================================================================== ## 
MIT License
Copyright (c) 2024 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
## =========================================================================== ## 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: ../Industrial_Robotics_Gym/__init__.py
## =========================================================================== ## 
"""

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
#       The mode called "Default" demonstrates an environment without a collision object.
#   'Collision-Free': 
#       The mode called "Collision-Free" demonstrates an environment with a collision object.
CONST_ENV_MODE = ['Default', 'Collision-Free']
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