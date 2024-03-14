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
File Name: ../Industrial_Robotics_Gym/Utilities.py
## =========================================================================== ## 
"""

def Get_Environment_ID(name: str, env_mode: int) -> str:
    """
    Description:
        Get a string of the gym environment ID for parameters defined by the function input.

    Args:
        (1) name [string]: Name of the robotic structure.
        (2) env_mode [int]: The name of the environment mode.
                                env_mode = 'Default' or 'Collision-Free'

    Returns:
        (1) parameter [string]: The string of the desired gym environment ID.
    """

    try:
        assert env_mode in ['Default', 'Collision-Free']

        return {
            'Universal_Robots_UR3': lambda env_m: f'Ur3-{env_m}-Reach-v0',
            'ABB_IRB_120': lambda env_m: f'AbbIrb120-{env_m}-Reach-v0',
            'ABB_IRB_120_L_Ax': lambda env_m: f'AbbIrb120L-{env_m}-Reach-v0',
            'ABB_IRB_14000_R': lambda env_m: f'AbbIrb14000R-{env_m}-Reach-v0',
            'ABB_IRB_14000_L': lambda env_m: f'AbbIrb14000L-{env_m}-Reach-v0',
            'EPSON_LS3_B401S': lambda env_m: f'EpsonLs3-{env_m}-Reach-v0'
        }[name](env_mode)
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] Incorrect environment mode selected. The selected mode must be chosen from the two options (Default, Collision-Free).')