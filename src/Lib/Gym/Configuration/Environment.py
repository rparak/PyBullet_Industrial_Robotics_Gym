# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

@dataclass
class Cuboid_Str:
    """
    Description:
        The auxiliary structure of the main parameters of the cuboid.

        Note:
            Private structure.
    """

    # Homogeneous transformation matrix of the cuboid.
    #   Unit [Matrix<float>]
    T: tp.List[tp.List[float]] = field(default_factory=list)
    # The size of the cuboid.
    #   Unit [Vector<float> 1x3]
    Size: tp.List[float] = field(default_factory=list)
    # The color of the cuboid.
    #   Note:
    #       Format: rgba(red, green, blue
    #   Unit [Vector<float> 1x3]
    Color: tp.List[float] = field(default_factory=list)

@dataclass
class Configuration_Space_Str:
    """
    Description:
        The structure of the main parameters of the environment configuration space.

    Initialization of the Class (structure):
        Input:
            (1) Name [string]: The name of the robotic structure for which 
                               the configuration space will be defined.

    Example:
        Initialization:
            Cls = Configuration_Space_Str(name)
            Cls.Name = ...
            ...
            Cls.Search = ..
    """

    # The name of the robotic structure for which 
    # the configuration space will be defined.
    #   Unit [string]
    Name: str = ''
    # The search (configuration) space indicates the place
    # where the robot can move freely.
    #   Unit [Cuboid_Str(object)]
    Search: Cuboid_Str = field(default_factory=Cuboid_Str)
    # The target (configuration) space indicates the place
    # where the robot aims to reach.
    #   Unit [Cuboid_Str(object)]
    Target: Cuboid_Str = field(default_factory=Cuboid_Str)

"""         
Description:
    Defined structure of the main parameters of the configuration 
    space for the individual robotic structures.
"""

# Universal Robots UR3.
Universal_Robots_UR3_C_Str = Configuration_Space_Str(Name='Universal_Robots_UR3')
Universal_Robots_UR3_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.3, 0.0, 0.15], dtype=np.float64)), 
                                               np.array([0.3, 0.4, 0.3], dtype=np.float64), [1.0, 1.0, 0.0])
Universal_Robots_UR3_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.30, 0.0, 0.1], dtype=np.float64)), 
                                               np.array([0.2, 0.3, 0.15], dtype=np.float64), [0.0, 1.0, 0.0])

# ABB IRB 120.
ABB_IRB_120_C_Str = Configuration_Space_Str(Name='ABB_IRB_120')
ABB_IRB_120_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.0, 0.225], dtype=np.float64)), 
                                      np.array([0.3, 0.4, 0.45], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.0, 0.1], dtype=np.float64)), 
                                      np.array([0.2, 0.3, 0.15], dtype=np.float64), [0.0, 1.0, 0.0])

# ABB IRB 120 with SMC Linear Axis (LEJSH63NZA 800).
ABB_IRB_120_L_Ax_C_Str = Configuration_Space_Str(Name='ABB_IRB_120_L_Ax')
ABB_IRB_120_L_Ax_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.4, 0.225 + 0.113], dtype=np.float64)), 
                                           np.array([0.3, 0.4, 0.45], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_L_Ax_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.4, 0.1 + 0.113], dtype=np.float64)), 
                                           np.array([0.2, 0.3, 0.15], dtype=np.float64), [0.0, 1.0, 0.0])

# ABB IRB 14000 (Right).
ABB_IRB_14000_R_C_Str = Configuration_Space_Str(Name='ABB_IRB_14000_R')
ABB_IRB_14000_R_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15, 0.125], dtype=np.float64)), 
                                          np.array([0.20, 0.30, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_R_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15, 0.075], dtype=np.float64)), 
                                          np.array([0.125, 0.225, 0.10], dtype=np.float64), [0.0, 1.0, 0.0])

# ABB IRB 14000 (Left).
ABB_IRB_14000_L_C_Str = Configuration_Space_Str(Name='ABB_IRB_14000_L')
ABB_IRB_14000_L_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15, 0.125], dtype=np.float64)), 
                                          np.array([0.20, 0.30, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_L_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15, 0.075], dtype=np.float64)), 
                                          np.array([0.125, 0.225, 0.10], dtype=np.float64), [0.0, 1.0, 0.0])

# Epson LS3-B401S.
EPSON_LS3_B401S_C_Str = Configuration_Space_Str(Name='EPSON_LS3_B401S')
EPSON_LS3_B401S_C_Str.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.0, 0.07], dtype=np.float64)), 
                                          np.array([0.20, 0.30, 0.14], dtype=np.float64), [1.0, 1.0, 0.0])
EPSON_LS3_B401S_C_Str.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.0, 0.05], dtype=np.float64)), 
                                          np.array([0.15, 0.25, 0.05], dtype=np.float64), [0.0, 1.0, 0.0])