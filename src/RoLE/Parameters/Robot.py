"""
## =========================================================================== ## 
MIT License
Copyright (c) 2023 Roman Parak
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
File Name: Robot.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#   ../RoLE/Primitives/Core
from RoLE.Primitives.Core import Box_Cls
#   ../RoLE/Primitives/Core
from RoLE.Collider.Core import AABB_Cls, OBB_Cls

@dataclass
class DH_Parameters_Str:
    """
    Description:
        The auxiliary structure of the Denavit-Hartenberg (DH) parameters.

        Note:
            DH (Denavit-Hartenberg) parameters: 
                (1) theta_zero [Vector<float>]: Joint angle (Theta_i). Rotation part in radians.
                                                Unit: [radian]                        
                (2) a [Vector<float>]: Link length (a_i). Translation part in meters.
                                       Unit: [meter]
                (3) d [Vector<float>]: Link offset (d_i). Translation part in meters.
                                       Unit: [meter]
                (4) alpha [Vector<float>]: Link twist (alpha_i). Rotation part in radians.
                                           Unit: [radian]
    """

    # Standard Denavit-Hartenberg (DH):
    #       DH_theta_zero = th{i} + theta_zero{i}
    #       DH_a          = a{i}
    #       DH_d          = d{i}
    #       DH_alpha      = alpha{i}
    #   Unit [Matrix<float>]
    Standard: tp.List[tp.List[float]] = field(default_factory=list)
    # Modified Denavit-Hartenberg (DH):
    #       DH_theta_zero = th{i} + theta_zero{i}
    #       DH_a          = a{i - 1}
    #       DH_d          = d{i}
    #       DH_alpha      = alpha{i - 1}
    #   Unit [Matrix<float>]
    Modified: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Theta_Parameters_Str(object):
    """
    Description:
        The auxiliary structure of the joint (theta) parameters.
    """

    # Zero absolute position of each joint.
    #   Unit [Vector<float>]
    Zero: tp.List[float] = field(default_factory=list)
    # Home absolute position of each joint.
    #   Unit [Vector<float>]
    Home: tp.List[float] = field(default_factory=list)
    # Limits of absolute joint position in radians and meters.
    #   Unit [Matrix<float>]
    Limit: tp.List[tp.List[float]] = field(default_factory=list)
    # Other parameters of the object structure.
    #   The name of the joints.
    #       Unit [Vector<string>]
    Name: tp.List[str] = field(default_factory=list)
    #   Identification of the type of joints.
    #       Note: R - Revolute, P - Prismatic
    #       Unit [Vector<string>]
    Type: tp.List[str] = field(default_factory=list)
    #   Identification of the axis of the absolute position of the joint. 
    #       Note: 'X', 'Z'
    #       Unit [Vector<string>]
    Axis: tp.List[str] = field(default_factory=list)
    #   Identification of the axis direction.
    #       Note: (+1) - Positive, (-1) - Negative
    #       Unit [Vector<int>]
    Direction: tp.List[int] = field(default_factory=list)

@dataclass
class T_Parameters_Str:
    """
    Description:
        The auxiliary structure of the homogeneous transformation matrix {T} parameters.
    """

    # Homogeneous transformation matrix of the base.
    #   Unit [Matrix<float>]
    Base: tp.List[tp.List[float]] = field(default_factory=list)
    # Homogeneous transformation matrix of the end-effector (tool).
    #   Unit [Matrix<float>]
    End_Effector: tp.List[tp.List[float]] = field(default_factory=list)
    # The zero configuration of the homogeneous transformation 
    # matrix of each joint (theta). The method (Standard, Modified) chosen 
    # to determine the configuration depends on the specific task.
    #   Unit [Matrix<float>]
    Zero_Cfg: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Collider_Str:
    """
    Description:
        The auxiliary structure of both the internal and external colliders.

        Note:
            Internal colliders are generated from the program, see below:
                ./src/Evaluation/Blender/Collider/gen_colliders.py
    """

    # Internal colliders of the base.
    #   Unit [Tuple<OBB_Cls(object)>]
    Base: tp.Tuple[OBB_Cls] = field(default_factory=tuple)
    # Internal colliders of the joints.
    #   Unit [Tuple<OBB_Cls(object)>]
    Theta: tp.Tuple[OBB_Cls] = field(default_factory=tuple)
    # Offset of the self-collision detection function.
    #   Note:
    #       If the offset is equal to 0, the function checks all 
    #       combinations of collisions.
    #   Unit [int]
    Offset: int = 0
    # External colliders.
    #   Unit [Tuple<AABB_Cls(object)>/Tuple<OBB_Cls(object)>]
    External: tp.Tuple[tp.Union[AABB_Cls, OBB_Cls]] = field(default_factory=tuple)
    # Optimized collision pairs.
    #   Note:
    #       The script to optimize collision pairs can be found here:
    #           ../Evaluation/Kinematics/Collider/optimize_collision_pairs.py
    #   Unit [Matrix<float>]
    Pairs: tp.List[tp.List[float]] = field(default_factory=list)

@dataclass
class Robot_Parameters_Str:
    """
    Description:
        The structure of the main parameters of the robot.

    Initialization of the Class (structure):
        Input:
            (1) Name [string]: Name of the robotic structure.
            (2) Id [int]: Identification number.

    Example:
        Initialization:
            Cls = Robot_Parameters_Str(name)
            Cls.Name = ...
            ...
            Cls.T = ..
    """

    # Name of the robotic structure.
    #   Unit [string]
    Name: str = ''
    # Identification number.
    #   Unit [int]
    Id: int = 0
    # Denavit-Hartenberg (DH) parameters.
    #   Unit [DH_Parameters_Str(object)]
    DH: DH_Parameters_Str = field(default_factory=DH_Parameters_Str)
    # Absolute joint position (theta) parameters.
    #   Unit [Theta_Parameters_Str(object)]
    Theta: Theta_Parameters_Str = field(default_factory=Theta_Parameters_Str)
    # Homogeneous transformation matrix (T) parameters.
    #   Unit [T_Parameters_Str(object)]
    T: T_Parameters_Str = field(default_factory=T_Parameters_Str)
    # Internal and external colliders of the robot structure.
    #   Unit [Collider_Str(object)]
    Collider: Collider_Str = field(default_factory=Collider_Str)
    # Information about whether the external axis is part of the robot 
    # or not. For example, a linear track.
    #   Unit [bool]
    External_Axis: bool = False

"""
Robot Type - Universal Robots UR3:
    Absolute Joint Position:
        Joint 1 - 6: [+/- 360.0] [°] -> modified [+/- 180.0] [°]  

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [   0.0,      0.0,      0.0,     0.0,     0.0,    0.0]
        a          = [   0.0, -0.24365, -0.21325,     0.0,     0.0,    0.0]
        d          = [0.1519,      0.0,      0.0, 0.11235, 0.08535, 0.0819]
        alpha      = [  1.57,      0.0,      0.0,    1.57,   -1.57,    0.0]
"""
Universal_Robots_UR3_Str = Robot_Parameters_Str(Name='Universal_Robots_UR3', Id=1)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
#   2\ Stand:
#       [[1.0, 0.0, 0.0,  0.0],
#        [0.0, 1.0, 0.0,  0.0],
#        [0.0, 0.0, 1.0, 0.02],
#        [0.0, 0.0, 0.0,  1.0]]
Universal_Robots_UR3_Str.T.Base = HTM_Cls(None, np.float64).Rotation(np.array([0.0, 0.0, 1.5707963267948966]), 'ZYX')
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
Universal_Robots_UR3_Str.T.End_Effector = HTM_Cls(None, np.float64)
# Denavit-Hartenberg (DH)
Universal_Robots_UR3_Str.DH.Standard = np.array([[0.0,      0.0,  0.1519,   1.5707963267948966],
                                                 [0.0, -0.24365,     0.0,                  0.0],
                                                 [0.0, -0.21325,     0.0,                  0.0],
                                                 [0.0,      0.0, 0.11235,   1.5707963267948966],
                                                 [0.0,      0.0, 0.08535,  -1.5707963267948966],
                                                 [0.0,      0.0,  0.0819,                  0.0]], dtype=np.float64) 
Universal_Robots_UR3_Str.DH.Modified = np.array([[0.0,      0.0,  0.1519,                  0.0],
                                                 [0.0,      0.0,     0.0,   1.5707963267948966],
                                                 [0.0, -0.24365,     0.0,                  0.0],
                                                 [0.0, -0.21325, 0.11235,                  0.0],
                                                 [0.0,      0.0, 0.08535,   1.5707963267948966],
                                                 [0.0,      0.0,  0.0819,  -1.5707963267948966]], dtype=np.float64)
# Zero/Home absolute position of each joint.
Universal_Robots_UR3_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)
Universal_Robots_UR3_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([-90.0, -90.0, 0.0, -90.0, 0.0, 0.0], 
                                                                            dtype=np.float64))
# Limits of absolute joint position.
Universal_Robots_UR3_Str.Theta.Limit = np.array([[-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793], 
                                                 [-3.141592653589793, 3.141592653589793]], dtype = np.float64)
# Other parameters of the robot structure.
Universal_Robots_UR3_Str.Theta.Name = [f'Joint_1_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}', 
                                       f'Joint_2_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}', 
                                       f'Joint_3_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}', 
                                       f'Joint_4_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}', 
                                       f'Joint_5_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}', 
                                       f'Joint_6_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}']
Universal_Robots_UR3_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R']
Universal_Robots_UR3_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z']
Universal_Robots_UR3_Str.Theta.Direction = np.array([1, 1, 1, 1, 1, 1], dtype=np.int8)
Universal_Robots_UR3_Str.External_Axis = False
# Colliders of the robot structure.
#   1\ Internal.
Universal_Robots_UR3_Str.Collider.Base = {f'Base_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00001, -0.04302], 
                                                                                                                                                [0.12807, 0.12802, 0.08605]))}
Universal_Robots_UR3_Str.Collider.Theta = {f'Joint_1_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([-0.00014, 0.00405, 0.00439], 
                                                                                                                                                    [0.09099, 0.09990, 0.12292])),
                                           f'Joint_2_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([0.11826, 0.00013, -0.11553], 
                                                                                                                                                    [0.32829, 0.09098, 0.12305])),
                                           f'Joint_3_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([0.10402, 0.00001, -0.02942], 
                                                                                                                                                    [0.28304, 0.07503, 0.09708])),
                                           f'Joint_4_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([-0.00001, -0.00078, 0.00610], 
                                                                                                                                                    [0.06444, 0.09067, 0.07681])),
                                           f'Joint_5_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00176, 0.00610], 
                                                                                                                                                    [0.06443, 0.08872, 0.07680])),
                                           f'Joint_6_Collider_{Universal_Robots_UR3_Str.Name}_ID_{Universal_Robots_UR3_Str.Id:03}': OBB_Cls(Box_Cls([-0.00007, 0.00326, 0.01962], 
                                                                                                                                                    [0.06298, 0.06961, 0.03936]))}
Universal_Robots_UR3_Str.Collider.Offset = 2
#   2\ External.
Universal_Robots_UR3_Str.Collider.External = {}
#   Collision pairs.
Universal_Robots_UR3_Str.Collider.Pairs = np.array([[0, 3], [0, 4], [0, 5], [0, 6], 
                                                    [1, 6], [2, 5], [2, 6], [3, 6]], dtype=np.int8)


"""
Robot Type - ABB IRB 120:
    Absolute Joint Position:
        Joint 1: [+/- 165.0] [°]
        Joint 2: [+/- 110.0] [°]
        Joint 3: [-110.0, +70.0] [°]
        Joint 4: [+/- 160.0] [°]
        Joint 5: [+/- 120.0] [°]
        Joint 6: [+/- 400.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [  0.0, -1.57,   0.0,   0.0,   0.0,   0.0]
        a          = [  0.0, 0.270,  0.07,   0.0,   0.0,   0.0]
        d          = [0.290,   0.0,   0.0, 0.302,   0.0, 0.072]
        alpha      = [-1.57,   0.0, -1.57,  1.57, -1.57,   0.0]
"""
ABB_IRB_120_Str = Robot_Parameters_Str(Name='ABB_IRB_120', Id=1)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_Str.T.Base = HTM_Cls(None, np.float64)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_Str.T.End_Effector = HTM_Cls(None, np.float64)
# Denavit-Hartenberg (DH)
ABB_IRB_120_Str.DH.Standard = np.array([[0.0,                   0.0, 0.290, -1.5707963267948966],
                                        [-1.5707963267948966, 0.270,   0.0,                 0.0],
                                        [0.0,                  0.07,   0.0, -1.5707963267948966],
                                        [0.0,                   0.0, 0.302,  1.5707963267948966],
                                        [0.0,                   0.0,   0.0, -1.5707963267948966],
                                        [3.141592653589793,     0.0, 0.072,                 0.0]], dtype = np.float64)
ABB_IRB_120_Str.DH.Modified = np.array([[0.0,                   0.0, 0.290,                 0.0],
                                        [-1.5707963267948966,   0.0,   0.0, -1.5707963267948966],
                                        [0.0,                 0.270,   0.0,                 0.0],
                                        [0.0,                  0.07, 0.302, -1.5707963267948966],
                                        [0.0,                   0.0,   0.0,  1.5707963267948966],
                                        [3.141592653589793,     0.0, 0.072, -1.5707963267948966]], dtype = np.float64)
# Zero/Home absolute position of each joint.
ABB_IRB_120_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)
ABB_IRB_120_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, 0.0, 0.0, 0.0, 90.0, 0.0], 
                                                                   dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_120_Str.Theta.Limit = np.array([[-2.8797932657906435, 2.8797932657906435], 
                                        [-1.9198621771937625, 1.9198621771937625], 
                                        [-1.9198621771937625, 1.2217304763960306], 
                                        [ -2.792526803190927,  2.792526803190927], 
                                        [-2.0943951023931953, 2.0943951023931953], 
                                        [ -3.141592653589793,  3.141592653589793]], dtype = np.float64)
# Other parameters of the robot structure.
ABB_IRB_120_Str.Theta.Name = [f'Joint_1_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}', 
                              f'Joint_2_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}', 
                              f'Joint_3_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}', 
                              f'Joint_4_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}', 
                              f'Joint_5_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}', 
                              f'Joint_6_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}']
ABB_IRB_120_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_120_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z']
ABB_IRB_120_Str.Theta.Direction = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.int8)
ABB_IRB_120_Str.External_Axis = False
# Colliders of the robot structure.
#   1\ Internal.
ABB_IRB_120_Str.Collider.Base = {f'Base_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([0.06060, 0.00000, -0.08317], 
                                                                                                                     [0.30120, 0.18052, 0.16633]))}
ABB_IRB_120_Str.Collider.Theta = {f'Joint_1_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.03631], 
                                                                                                                         [0.21500, 0.21700, 0.19738])),
                                  f'Joint_2_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([-0.12301, 0.00000, 0.00025], 
                                                                                                                         [0.38941, 0.14486, 0.21926])),
                                  f'Joint_3_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([-0.04557, -0.03259, -0.00175], 
                                                                                                                         [0.18803, 0.24401, 0.11550])),
                                  f'Joint_4_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([-0.00700, -0.00047, 0.05666], 
                                                                                                                         [0.10591, 0.12694, 0.19348])),
                                  f'Joint_5_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, -0.00006, 0.00000], 
                                                                                                                         [0.08200, 0.13189, 0.06962])),
                                  f'Joint_6_Collider_{ABB_IRB_120_Str.Name}_ID_{ABB_IRB_120_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.00650], 
                                                                                                                         [0.04000, 0.03983, 0.01300]))}
ABB_IRB_120_Str.Collider.Offset = 2
#   2\ External.
ABB_IRB_120_Str.Collider.External = {}
#   Collision pairs.
ABB_IRB_120_Str.Collider.Pairs = np.array([[0, 4], [0, 5], [0, 6], [0, 3], 
                                           [1, 4], [1, 5], [1, 6]], dtype=np.int8)

"""
Robot Type - ABB IRB 120 with SMC Linear Axis (LEJSH63NZA 800):
    Absolute Joint Position:
        Joint L: [-0.8, +0.0] [m]
        Joint 1: [+/- 165.0] [°]
        Joint 2: [+/- 110.0] [°]
        Joint 3: [-110.0, +70.0] [°]
        Joint 4: [+/- 160.0] [°]
        Joint 5: [+/- 120.0] [°]
        Joint 6: [+/- 400.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [  0.0,   0.0, -1.57,   0.0,   0.0,   0.0,   0.0]
        a          = [  0.0,   0.0, 0.270,  0.07,   0.0,   0.0,   0.0]
        d          = [0.113, 0.290,   0.0,   0.0, 0.302,   0.0, 0.072]
        alpha      = [  0.0, -1.57,   0.0, -1.57,  1.57, -1.57,   0.0]
"""
ABB_IRB_120_L_Ax_Str = Robot_Parameters_Str(Name='ABB_IRB_120_L_Ax', Id=1)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_L_Ax_Str.T.Base = HTM_Cls(None, np.float64).Rotation(np.array([0.0, 0.0, -1.5707963267948966]), 'ZYX')
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_120_L_Ax_Str.T.End_Effector = HTM_Cls(None, np.float64)
# Denavit-Hartenberg (DH)
ABB_IRB_120_L_Ax_Str.DH.Standard = np.array([[0.0,                   0.0, 0.113,                 0.0],
                                             [0.0,                   0.0, 0.290, -1.5707963267948966],
                                             [-1.5707963267948966, 0.270,   0.0,                 0.0],
                                             [0.0,                  0.07,   0.0, -1.5707963267948966],
                                             [0.0,                   0.0, 0.302,  1.5707963267948966],
                                             [0.0,                   0.0,   0.0, -1.5707963267948966],
                                             [3.141592653589793,     0.0, 0.072,                 0.0]], dtype = np.float64)
ABB_IRB_120_L_Ax_Str.DH.Modified = np.array([[0.0,                   0.0, 0.113,                 0.0],
                                             [0.0,                   0.0, 0.290,                 0.0],
                                             [-1.5707963267948966,   0.0,   0.0, -1.5707963267948966],
                                             [0.0,                 0.270,   0.0,                 0.0],
                                             [0.0,                  0.07, 0.302, -1.5707963267948966],
                                             [0.0,                   0.0,   0.0,  1.5707963267948966],
                                             [3.141592653589793,     0.0, 0.072, -1.5707963267948966]], dtype = np.float64)
# Zero/Home absolute position of each joint.
ABB_IRB_120_L_Ax_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float64)
ABB_IRB_120_L_Ax_Str.Theta.Home = np.append([0.4], Mathematics.Degree_To_Radian(np.array([90.0, 0.0, 0.0, 0.0, 90.0, 0.0], 
                                                                                         dtype=np.float64)))
# Limits of absolute joint position.
ABB_IRB_120_L_Ax_Str.Theta.Limit = np.array([[                0.0,              0.800],
                                             [-2.8797932657906435, 2.8797932657906435], 
                                             [-1.9198621771937625, 1.9198621771937625], 
                                             [-1.9198621771937625, 1.2217304763960306], 
                                             [ -2.792526803190927,  2.792526803190927], 
                                             [-2.0943951023931953, 2.0943951023931953], 
                                             [ -3.141592653589793,  3.141592653589793]], dtype = np.float64)
# Other parameters of the robot structure.
ABB_IRB_120_L_Ax_Str.Theta.Name = [f'Joint_L_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}', 
                                   f'Joint_1_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}',
                                   f'Joint_2_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}', 
                                   f'Joint_3_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}', 
                                   f'Joint_4_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}', 
                                   f'Joint_5_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}', 
                                   f'Joint_6_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}']
ABB_IRB_120_L_Ax_Str.Theta.Type = ['P', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_120_L_Ax_Str.Theta.Axis = ['X', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']
ABB_IRB_120_L_Ax_Str.Theta.Direction = np.array([-1, 1, 1, 1, 1, 1, 1], dtype=np.int8)
ABB_IRB_120_L_Ax_Str.External_Axis = True
# Colliders of the robot structure.
#   1\ Internal.
ABB_IRB_120_L_Ax_Str.Collider.Base = {f'Base_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([0.33848, 0.03402, -0.05515], 
                                                                                                                                    [1.18700, 0.32000, 0.11030])),
                                      f'Base_1_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([0.06060, 0.00000, -0.08317], 
                                                                                                                                      [0.30120, 0.18052, 0.16633]))}
ABB_IRB_120_L_Ax_Str.Collider.Theta = {f'Joint_L_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([-0.00052, 0.03000, 0.01960], 
                                                                                                                                        [0.18000, 0.24000, 0.03920])),
                                       f'Joint_1_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.03631], 
                                                                                                                                        [0.21500, 0.21700, 0.19738])),
                                       f'Joint_2_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([-0.12301, 0.00000, 0.00025], 
                                                                                                                                        [0.38941, 0.14486, 0.21926])),
                                       f'Joint_3_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([-0.04557, -0.03259, -0.00175], 
                                                                                                                                        [0.18803, 0.24401, 0.11550])),
                                       f'Joint_4_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([-0.00700, -0.00047, 0.05666], 
                                                                                                                                        [0.10591, 0.12694, 0.19348])),
                                       f'Joint_5_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, -0.00006, 0.00000], 
                                                                                                                                        [0.08200, 0.13189, 0.06962])),
                                       f'Joint_6_Collider_{ABB_IRB_120_L_Ax_Str.Name}_ID_{ABB_IRB_120_L_Ax_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.00650], 
                                                                                                                                        [0.04000, 0.03983, 0.01300]))}
ABB_IRB_120_L_Ax_Str.Collider.Offset = 2
#   2\ External.
ABB_IRB_120_L_Ax_Str.Collider.External = {}
#   Collision pairs.
ABB_IRB_120_L_Ax_Str.Collider.Pairs = np.array([[0, 6], [0, 7], [0, 8], [0, 5], [1, 6], [1, 7], [1, 8], 
                                                [2, 6], [2, 7], [2, 8], [2, 5], [3, 6], [3, 7], [3, 8]], dtype=np.int8)

"""
Robot Type - ABB IRB 14000 (Right):
    Absolute Joint Position:
        Joint 1: [+/- 168.5] [°]
        Joint 2: [-143.5, +43.5] [°]
        Joint 7: [+/- 168.5] [°]
        Joint 3: [-123.5, +80.0] [°]
        Joint 4: [+/- 290.0] [°]
        Joint 5: [-88.0, +138.0] [°]
        Joint 6: [+/- 229.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [   0.0,   0.0,    0.0,  -1.57,  3.14,    0.0,   0.0]
        a          = [-0.030, 0.030, 0.0405, 0.0405, 0.027, -0.027,   0.0]
        d          = [   0.1,   0.0, 0.2515,    0.0, 0.265,    0.0, 0.036]
        alpha      = [ -1.57,  1.57,  -1.57,  -1.57, -1.57,   1.57,   0.0]
"""
ABB_IRB_14000_R_Str = Robot_Parameters_Str(Name='ABB_IRB_14000_R', Id=1)
# Homogeneous transformation matrix of the base of the right arm (T_Base @ T_Base_R)
#   1\ Right Arm:
#       [[ 0.5713, -0.1071,  0.8138,  0.0536],
#        [ 0.6198,  0.7063, -0.3421, -0.0725],
#        [-0.5381,  0.6998,  0.4698,  0.4149],
#        [    0.0,     0.0,     0.0,     1.0]]
ABB_IRB_14000_R_Str.T.Base = HTM_Cls([[ 0.5713, -0.1071,  0.8138,  0.0536],
                                      [ 0.6198,  0.7063, -0.3421, -0.0725],
                                      [-0.5381,  0.6998,  0.4698,  0.4149],
                                      [    0.0,     0.0,     0.0,     1.0]], np.float64)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_14000_R_Str.T.End_Effector = HTM_Cls([[1.0, 0.0, 0.0, 0.0],
                                              [0.0, 1.0, 0.0, 0.0],
                                              [0.0, 0.0, 1.0, 0.0],
                                              [0.0, 0.0, 0.0, 1.0]], np.float64)
# Denavit-Hartenberg (DH)
ABB_IRB_14000_R_Str.DH.Standard = np.array([[                0.0, -0.030,    0.1, -1.5707963267948966],
                                            [                0.0,  0.030,    0.0,  1.5707963267948966],
                                            [                0.0, 0.0405, 0.2515, -1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793,  0.027,  0.265, -1.5707963267948966],
                                            [                0.0, -0.027,    0.0,  1.5707963267948966],
                                            [                0.0,    0.0,  0.036,                 0.0]], dtype = np.float64)
ABB_IRB_14000_R_Str.DH.Modified = np.array([[                0.0,    0.0,    0.1,                 0.0],
                                            [                0.0, -0.030,    0.0, -1.5707963267948966],
                                            [                0.0,  0.030, 0.2515,  1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793, 0.0405,  0.265, -1.5707963267948966],
                                            [                0.0,  0.027,    0.0, -1.5707963267948966],
                                            [                0.0, -0.027,  0.036,  1.5707963267948966]], dtype = np.float64)
# Zero/Home absolute position of each joint.
ABB_IRB_14000_R_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float64)
ABB_IRB_14000_R_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, -130.0, -135.0, 30.0, 0.0, 40.0, 0.0], 
                                                                       dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_14000_R_Str.Theta.Limit = np.array([[-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.504547476611863, 0.7592182246175333],
                                            [-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.155481626212997, 1.3962634015954636], 
                                            [ -5.061454830783556,  5.061454830783556], 
                                            [  -1.53588974175501, 2.4085543677521746], 
                                            [-3.9968039870670147, 3.9968039870670147]], dtype = np.float64)
# Other parameters of the robot structure.
ABB_IRB_14000_R_Str.Theta.Name = [f'Joint_1_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_2_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_7_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_3_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_4_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_5_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}', 
                                  f'Joint_6_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}']
ABB_IRB_14000_R_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_14000_R_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']
ABB_IRB_14000_R_Str.Theta.Direction = np.array([1, 1, 1, 1, 1, 1, 1], dtype=np.int8)
ABB_IRB_14000_R_Str.External_Axis = False
# Colliders of the robot structure.
#   1\ Internal.
ABB_IRB_14000_R_Str.Collider.Base = {f'Base_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.00000], 
                                                                                                                                 [0.00200, 0.00200, 0.00200])),
                                     f'Base_Collider_ID_1_ABB_IRB_14000_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.17702, -0.00069, 0.05030], 
                                                                                                                         [0.35605, 0.31870, 0.09939])),
                                     f'Base_Collider_ID_2_ABB_IRB_14000_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.07988, 0.14999, -0.04014], 
                                                                                                                         [0.42757, 0.09972, 0.07875])),
                                     f'Base_Collider_ID_3_ABB_IRB_14000_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.07987, -0.14999, -0.04014], 
                                                                                                                         [0.42758, 0.09976, 0.07875])),
                                     f'Base_Collider_ID_4_ABB_IRB_14000_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.24378, 0.00002, -0.18980], 
                                                                                                                         [0.23212, 0.22006, 0.22057])),
                                     f'Base_Collider_ID_5_ABB_IRB_14000_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.21577, -0.00001, -0.38543], 
                                                                                                                         [0.29054, 0.25038, 0.17085]))}
ABB_IRB_14000_R_Str.Collider.Theta = {f'Joint_1_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.00867, -0.00780, 0.04144], 
                                                                                                                                     [0.14013, 0.14044, 0.18074])),
                                      f'Joint_2_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([-0.01831, 0.06249, 0.01221], 
                                                                                                                                     [0.13314, 0.22109, 0.13158])), 
                                      f'Joint_7_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([-0.02064, -0.01583, 0.03952], 
                                                                                                                                     [0.13622, 0.12657, 0.17557])), 
                                      f'Joint_3_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([-0.01870, -0.05825, 0.01533], 
                                                                                                                                     [0.13341, 0.21283, 0.12563])), 
                                      f'Joint_4_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([-0.00856, -0.01428, 0.05104], 
                                                                                                                                     [0.10046, 0.11180, 0.16608])), 
                                      f'Joint_5_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.01735, -0.01680, 0.01499], 
                                                                                                                                     [0.10141, 0.09905, 0.12565])), 
                                      f'Joint_6_Collider_{ABB_IRB_14000_R_Str.Name}_ID_{ABB_IRB_14000_R_Str.Id:03}': OBB_Cls(Box_Cls([0.00017, 0.00020, 0.01621], 
                                                                                                                                     [0.06302, 0.06302, 0.04352]))}
ABB_IRB_14000_R_Str.Collider.Offset = 2
#   2\ External.
ABB_IRB_14000_R_Str.Collider.External = {}
#   Collision pairs.
ABB_IRB_14000_R_Str.Collider.Pairs = np.array([[1, 12], [1, 11], [1, 10], [2, 10], [2, 12], [2, 9], [2, 11], [3, 11], [3, 9], [3, 10], [3, 12], [4, 9], [4, 12], 
                                               [4, 11], [4, 10], [5, 10], [5, 8], [5, 9], [5, 11], [5, 12], [6, 12], [6, 11], [6, 10], [7, 12], [7, 11], [7, 10]], dtype=np.int8)

"""
Robot Type - ABB IRB 14000 (Left):
    Absolute Joint Position:
        Joint 1: [+/- 168.5] [°]
        Joint 2: [-143.5, +43.5] [°]
        Joint 7: [+/- 168.5] [°]
        Joint 3: [-123.5, +80.0] [°]
        Joint 4: [+/- 290.0] [°]
        Joint 5: [-88.0, +138.0] [°]
        Joint 6: [+/- 229.0] [°]

    Denavit-Hartenberg (DH) Standard:
        theta_zero = [   0.0,   0.0,    0.0,  -1.57,  3.14,    0.0,   0.0]
        a          = [-0.030, 0.030, 0.0405, 0.0405, 0.027, -0.027,   0.0]
        d          = [   0.1,   0.0, 0.2515,    0.0, 0.265,    0.0, 0.036]
        alpha      = [ -1.57,  1.57,  -1.57,  -1.57, -1.57,   1.57,   0.0]
"""
ABB_IRB_14000_L_Str = Robot_Parameters_Str(Name='ABB_IRB_14000_L', Id=1)
# Homogeneous transformation matrix of the base of the left arm (T_Base @ T_Base_L)
#   1\ Left Arm:
#       [[ 0.5716,  0.1048, 0.8138, 0.0536],
#        [-0.6170,  0.7088, 0.3420, 0.0725],
#        [-0.5410, -0.6976, 0.4698, 0.4149],
#        [    0.0,     0.0,    0.0,    1.0]]
ABB_IRB_14000_L_Str.T.Base = HTM_Cls([[ 0.5716,  0.1048, 0.8138, 0.0536],
                                      [-0.6170,  0.7088, 0.3420, 0.0725],
                                      [-0.5410, -0.6976, 0.4698, 0.4149],
                                      [    0.0,     0.0,    0.0,    1.0]], np.float64)
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
ABB_IRB_14000_L_Str.T.End_Effector = HTM_Cls(None, np.float64)
# Denavit-Hartenberg (DH)
ABB_IRB_14000_L_Str.DH.Standard = np.array([[                0.0, -0.030,    0.1, -1.5707963267948966],
                                            [                0.0,  0.030,    0.0,  1.5707963267948966],
                                            [                0.0, 0.0405, 0.2515, -1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793,  0.027,  0.265, -1.5707963267948966],
                                            [                0.0, -0.027,    0.0,  1.5707963267948966],
                                            [                0.0,    0.0,  0.036,                 0.0]], dtype = np.float64)
ABB_IRB_14000_L_Str.DH.Modified = np.array([[                0.0,    0.0,    0.1,                 0.0],
                                            [                0.0, -0.030,    0.0, -1.5707963267948966],
                                            [                0.0,  0.030, 0.2515,  1.5707963267948966],
                                            [-1.5707963267948966, 0.0405,    0.0, -1.5707963267948966],
                                            [  3.141592653589793, 0.0405,  0.265, -1.5707963267948966],
                                            [                0.0,  0.027,    0.0, -1.5707963267948966],
                                            [                0.0, -0.027,  0.036,  1.5707963267948966]], dtype = np.float64)
# Zero/Home absolute position of each joint.
ABB_IRB_14000_L_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float64)
ABB_IRB_14000_L_Str.Theta.Home = Mathematics.Degree_To_Radian(np.array([0.0, -130.0, 135.0, 30.0, 0.0, 40.0, 0.0], 
                                                                       dtype=np.float64))
# Limits of absolute joint position.
ABB_IRB_14000_L_Str.Theta.Limit = np.array([[-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.504547476611863, 0.7592182246175333],
                                            [-2.9408797896104453, 2.9408797896104453], 
                                            [ -2.155481626212997, 1.3962634015954636], 
                                            [ -5.061454830783556,  5.061454830783556], 
                                            [  -1.53588974175501, 2.4085543677521746], 
                                            [-3.9968039870670147, 3.9968039870670147]], dtype = np.float64)
# Parameters of the object (Blender robot arm).
ABB_IRB_14000_L_Str.Theta.Name = [f'Joint_1_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_2_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_7_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_3_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_4_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_5_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}', 
                                  f'Joint_6_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}']
ABB_IRB_14000_L_Str.Theta.Type = ['R', 'R', 'R', 'R', 'R', 'R', 'R']
ABB_IRB_14000_L_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z', 'Z', 'Z', 'Z']
ABB_IRB_14000_L_Str.Theta.Direction = np.array([1, 1, 1, 1, 1, 1, 1], dtype=np.int8)
ABB_IRB_14000_L_Str.External_Axis = False
# Colliders of the robot structure.
#   1\ Internal.
ABB_IRB_14000_L_Str.Collider.Base = {f'Base_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.00000], 
                                                                                                                                 [0.00200, 0.00200, 0.00200])),
                                     f'Base_Collider_ID_1_ABB_IRB_14000_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.17702, -0.00069, 0.05030], 
                                                                                                                         [0.35605, 0.31870, 0.09939])),
                                     f'Base_Collider_ID_2_ABB_IRB_14000_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.07988, 0.14999, -0.04014], 
                                                                                                                         [0.42757, 0.09972, 0.07875])),
                                     f'Base_Collider_ID_3_ABB_IRB_14000_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.07987, -0.14999, -0.04014], 
                                                                                                                         [0.42758, 0.09976, 0.07875])),
                                     f'Base_Collider_ID_4_ABB_IRB_14000_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.24378, 0.00002, -0.18980], 
                                                                                                                         [0.23212, 0.22006, 0.22057])),
                                     f'Base_Collider_ID_5_ABB_IRB_14000_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.21577, -0.00001, -0.38543], 
                                                                                                                         [0.29054, 0.25038, 0.17085]))}
ABB_IRB_14000_L_Str.Collider.Theta = {f'Joint_1_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.00867, -0.00780, 0.04144], 
                                                                                                                                     [0.14013, 0.14044, 0.18074])),
                                      f'Joint_2_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([-0.01831, 0.06249, 0.01221], 
                                                                                                                                     [0.13314, 0.22109, 0.13158])), 
                                      f'Joint_7_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([-0.02064, -0.01583, 0.03952], 
                                                                                                                                     [0.13622, 0.12657, 0.17557])), 
                                      f'Joint_3_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([-0.01870, -0.05825, 0.01533], 
                                                                                                                                     [0.13341, 0.21283, 0.12563])), 
                                      f'Joint_4_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([-0.00856, -0.01428, 0.05104], 
                                                                                                                                     [0.10046, 0.11180, 0.16608])), 
                                      f'Joint_5_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.01735, -0.01680, 0.01499], 
                                                                                                                                     [0.10141, 0.09905, 0.12565])), 
                                      f'Joint_6_Collider_{ABB_IRB_14000_L_Str.Name}_ID_{ABB_IRB_14000_L_Str.Id:03}': OBB_Cls(Box_Cls([0.00017, 0.00020, 0.01621], 
                                                                                                                                     [0.06302, 0.06302, 0.04352]))}
ABB_IRB_14000_L_Str.Collider.Offset = 2
#   2\ External.
ABB_IRB_14000_L_Str.Collider.External = {}
#   Collision pairs.

ABB_IRB_14000_L_Str.Collider.Pairs = np.array([[1, 10], [1, 12], [1, 11], [2, 11], [2, 10], [2, 12], [2, 9], [3, 10], [3, 12], [3, 11], [3, 9], [4, 10], [4, 9], 
                                               [4, 11], [4, 12], [5, 11], [5, 9], [5, 8], [5, 10], [5, 12], [6, 10], [6, 11], [6, 12], [7, 11], [7, 10], [7, 12]], dtype=np.int8)

"""
Robot Type - Epson LS3-B401S:
    Absolute Joint Position:
        Joint 1: [+/- 131.0] [°]
        Joint 2: [+/- 142.0] [°]
        Joint 3: [-0.150, +0.0] [m]
        Joint 4: [+/- 180.0] [°]

    Denavit-Hartenberg (DH) Standard:
        Method 1 (th_3 - rotates counterclockwise):
            Note 1: The direction of the Z axis is upwards.
            Note 2: The Denavit-Hartenberg parameter d from 
                    the table will be positive (see Kinematics.py).
                theta_zero = [   0.0,    0.0, 0.0,     0.0]
                a          = [ 0.225,  0.175, 0.0,     0.0]
                d          = [0.1731, 0.0499, 0.0, -0.0785]
                alpha      = [   0.0,    0.0, 0.0,     0.0]
        Method 2 (th_3 - rotates clockwise):
            Note 1: The direction of the Z axis is downwards.
            Note 2: The Denavit-Hartenberg parameter d from 
                    the table will be negative (see Kinematics.py).
                theta_zero = [   0.0,    0.0, 0.0,    0.0]
                a          = [ 0.225,  0.175, 0.0,    0.0]
                d          = [0.1731, 0.0499, 0.0, 0.0785]
                alpha      = [   0.0,   3.14, 0.0,    0.0]
"""
EPSON_LS3_B401S_Str = Robot_Parameters_Str(Name='EPSON_LS3_B401S', Id=1)
# Homogeneous transformation matrix of the base.
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
EPSON_LS3_B401S_Str.T.Base = HTM_Cls(None, np.float64).Rotation(np.array([0.0, 0.0, -1.5707963267948966]), 'ZYX')
# End-effector (tool):
#   1\ None: Identity Matrix
#       [[1.0, 0.0, 0.0, 0.0],
#        [0.0, 1.0, 0.0, 0.0],
#        [0.0, 0.0, 1.0, 0.0],
#        [0.0, 0.0, 0.0, 1.0]]
EPSON_LS3_B401S_Str.T.End_Effector = HTM_Cls(None, np.float64)
# Denavit-Hartenberg (DH)
EPSON_LS3_B401S_Str.DH.Standard = np.array([[0.0, 0.225,  0.1731,               0.0],
                                            [0.0, 0.175,  0.0499, 3.141592653589793],
                                            [0.0,   0.0,     0.0,               0.0],
                                            [0.0,   0.0,  0.0785,               0.0]], dtype = np.float64) 
EPSON_LS3_B401S_Str.DH.Modified = np.array([[0.0,   0.0,  0.1731,               0.0],
                                            [0.0, 0.225,  0.0499,               0.0],
                                            [0.0, 0.175,     0.0, 3.141592653589793],
                                            [0.0,   0.0,  0.0785,               0.0]], dtype = np.float64) 
# Zero/Home absolute position of each joint.
EPSON_LS3_B401S_Str.Theta.Zero = np.array([0.0, 0.0, 0.0, 0.0], 
                                          dtype = np.float64)
EPSON_LS3_B401S_Str.Theta.Home = np.array([Mathematics.Degree_To_Radian(90.0), Mathematics.Degree_To_Radian(0.0), 0.0, Mathematics.Degree_To_Radian(0.0)],
                                          dtype = np.float64)
# Limits of absolute joint position.
EPSON_LS3_B401S_Str.Theta.Limit = np.array([[  -2.303834612632515,  2.303834612632515], 
                                            [ -2.4609142453120048, 2.4609142453120048], 
                                            [                 0.0,              0.150], 
                                            [  -3.141592653589793,  3.141592653589793]], dtype = np.float64)
# Parameters of the object (Blender robot arm).
EPSON_LS3_B401S_Str.Theta.Name = [f'Joint_1_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}', 
                                  f'Joint_2_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}', 
                                  f'Joint_3_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}', 
                                  f'Joint_4_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}']
EPSON_LS3_B401S_Str.Theta.Type = ['R', 'R', 'P', 'R']
EPSON_LS3_B401S_Str.Theta.Axis = ['Z', 'Z', 'Z', 'Z']
EPSON_LS3_B401S_Str.Theta.Direction = np.array([1, 1, -1, 1], dtype=np.int8)
EPSON_LS3_B401S_Str.External_Axis = False
# Colliders of the robot structure.
#   1\ Internal.
EPSON_LS3_B401S_Str.Collider.Base = {f'Base_Collider_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.05875, -0.09830], 
                                                                                                                                 [0.14000, 0.22650, 0.19660]))}
EPSON_LS3_B401S_Str.Collider.Theta = {f'Joint_1_Collider_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}': OBB_Cls(Box_Cls([-0.11001, 0.00000, -0.01900], 
                                                                                                                                     [0.31957, 0.09958, 0.05400])),
                                      f'Joint_2_Collider_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}': OBB_Cls(Box_Cls([-0.08931, 0.00000, -0.10894], 
                                                                                                                                     [0.26735, 0.13073, 0.28968])),
                                      f'Joint_3_Collider_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}': OBB_Cls(Box_Cls([0.00000, 0.00000, 0.00000], 
                                                                                                                                     [0.00200, 0.00200, 0.00200])),
                                      f'Joint_4_Collider_{EPSON_LS3_B401S_Str.Name}_ID_{EPSON_LS3_B401S_Str.Id:03}': OBB_Cls(Box_Cls([0.00001, 0.00000, 0.16500], 
                                                                                                                                     [0.02998, 0.02999, 0.33000]))}
EPSON_LS3_B401S_Str.Collider.Offset = 2
#   2\ External.
EPSON_LS3_B401S_Str.Collider.External = {}
#   Collision pairs.
EPSON_LS3_B401S_Str.Collider.Pairs = np.array([[0, 4], [0, 3]], dtype=np.int8)
