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
class Collision_Object_Str:
    """
    Description:
        The auxiliary structure of the main parameters of the collision object.
    """
        
    # Homogeneous transformation matrix of the cuboid.
    #   Unit [Matrix<float>]
    T: tp.List[tp.List[float]] = field(default_factory=list)
    # The scale factor of the object.
    #   Unit [float]
    Scale: float = 0.0
    # The color of the object. Format: rgba(red, green, blue, alpha).
    #   Unit [Vector<float> 1x4]
    Color: tp.List[float] = field(default_factory=list)
    # Type of collision object. 
    #   Note:
    #       Type = 'Cube' or 'Sphere'
    #   Unit [string]
    Type: str = ''


@dataclass
class Configuration_Space_Str:
    """
    Description:
        The auxiliary structure of the main parameters of the configuration space.
    """

    # The search (configuration) space indicates the place
    # where the robot can move freely.
    #   Unit [Cuboid_Str(object)]
    Search: Cuboid_Str = field(default_factory=Cuboid_Str)
    # The target (configuration) space indicates the place
    # where the robot aims to reach.
    #   Unit [Cuboid_Str(object)]
    Target: Cuboid_Str = field(default_factory=Cuboid_Str)

@dataclass
class Environment_Str:
    """
    Description:
        The structure of the main parameters of the environment.

    Initialization of the Class (structure):
        Input:
            (1) Name [string]: The name of the robotic structure for which 
                               the environment will be defined.

    Example:
        Initialization:
            Cls = Environment_Str(name)
            Cls.Name = ...
            ...
            Cls.C.Search = ..
    """
        
    # The name of the robotic structure for which 
    # the configuration space will be defined.
    #   Unit [string]
    Name: str = ''
    # The main parameters of the configuration space.
    #   Unit [Configuration_Space_Str(object)]
    C: Configuration_Space_Str = field(default_factory=Configuration_Space_Str)
    # The main parameters of the collision object.
    #   Unit [None or Collision_Object_Str(object)]
    Collision_Object: Collision_Object_Str = field(default_factory=Collision_Object_Str)

"""         
Description:
    Defined structure of the main parameters of the configuration 
    space for the individual robotic structures.

    Environmnet ID:
        0: Configuration space without collision objects.
        1: Configuration space with one collision object.
"""

# Universal Robots UR3.
#   Environmnet ID 0.
Universal_Robots_UR3_Env_ID_0_Str = Environment_Str(Name='Universal_Robots_UR3')
Universal_Robots_UR3_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.30, 0.0, 0.150 + 0.02], dtype=np.float64)), 
                                                        np.array([0.20, 0.30, 0.30], dtype=np.float64), [1.0, 1.0, 0.0])
Universal_Robots_UR3_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.30, 0.0, 0.075 + 0.02], dtype=np.float64)), 
                                                        np.array([0.15, 0.25, 0.10], dtype=np.float64), [0.0, 1.0, 0.0])
Universal_Robots_UR3_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
Universal_Robots_UR3_Env_ID_1_Str = Environment_Str(Name='Universal_Robots_UR3')
Universal_Robots_UR3_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.30, 0.0, 0.150 + 0.02], dtype=np.float64)), 
                                                        np.array([0.20, 0.30, 0.30], dtype=np.float64), [1.0, 1.0, 0.0])
Universal_Robots_UR3_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.30, 0.1 - 0.02, 0.075], dtype=np.float64)), 
                                                        np.array([0.15, 0.10, 0.075], dtype=np.float64), [0.0, 1.0, 0.0])
Universal_Robots_UR3_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.30, 0.1 - 0.02, 0.075])
Universal_Robots_UR3_Env_ID_1_Str.Collision_Object.Scale = 0.025
Universal_Robots_UR3_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
Universal_Robots_UR3_Env_ID_1_Str.Collision_Object.Type = 'Sphere'

# ABB IRB 120.
#   Environmnet ID 0.
ABB_IRB_120_Env_ID_0_Str = Environment_Str(Name='ABB_IRB_120')
ABB_IRB_120_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.0, 0.225 + 0.02], dtype=np.float64)), 
                                               np.array([0.20, 0.30, 0.40], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.0, 0.120 + 0.02], dtype=np.float64)), 
                                               np.array([0.15, 0.25, 0.15], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_120_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
ABB_IRB_120_Env_ID_1_Str = Environment_Str(Name='ABB_IRB_120')
ABB_IRB_120_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.0, 0.225 + 0.02], dtype=np.float64)), 
                                               np.array([0.20, 0.30, 0.40], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.1 - 0.02, 0.10], dtype=np.float64)), 
                                               np.array([0.15, 0.10, 0.075], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_120_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.35, 0.1 - 0.02, 0.10])
ABB_IRB_120_Env_ID_1_Str.Collision_Object.Scale = 0.025
ABB_IRB_120_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
ABB_IRB_120_Env_ID_1_Str.Collision_Object.Type = 'Sphere'

# ABB IRB 120 with SMC Linear Axis (LEJSH63NZA 800)
#   Environmnet ID 0.
ABB_IRB_120_L_Ax_Env_ID_0_Str = Environment_Str(Name='ABB_IRB_120_L_Ax')
ABB_IRB_120_L_Ax_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.4, 0.225 + 0.113 + 0.02], dtype=np.float64)), 
                                                    np.array([0.20, 0.30, 0.40], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_L_Ax_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.4, 0.120 + 0.113 + 0.02], dtype=np.float64)), 
                                                    np.array([0.15, 0.25, 0.15], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_120_L_Ax_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
ABB_IRB_120_L_Ax_Env_ID_1_Str = Environment_Str(Name='ABB_IRB_120_L_Ax')
ABB_IRB_120_L_Ax_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.4, 0.225 + 0.113 + 0.02], dtype=np.float64)), 
                                                    np.array([0.20, 0.30, 0.40], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_120_L_Ax_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.1 - 0.02 + 0.4, 0.113 + 0.10], dtype=np.float64)), 
                                                    np.array([0.15, 0.10, 0.075], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_120_L_Ax_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.35, 0.1 - 0.02 + 0.4, 0.113 + 0.10])
ABB_IRB_120_L_Ax_Env_ID_1_Str.Collision_Object.Scale = 0.02
ABB_IRB_120_L_Ax_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
ABB_IRB_120_L_Ax_Env_ID_1_Str.Collision_Object.Type = 'Sphere'

# ABB IRB 14000 (Right).
#   Environmnet ID 0.
ABB_IRB_14000_R_Env_ID_0_Str = Environment_Str(Name='ABB_IRB_14000_R')
ABB_IRB_14000_R_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15, 0.125 + 0.02], dtype=np.float64)), 
                                                   np.array([0.175, 0.275, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_R_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15, 0.075 + 0.02], dtype=np.float64)), 
                                                   np.array([0.125, 0.225, 0.10], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_14000_R_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
ABB_IRB_14000_R_Env_ID_1_Str = Environment_Str(Name='ABB_IRB_14000_R')
ABB_IRB_14000_R_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15, 0.125 + 0.02], dtype=np.float64)), 
                                                   np.array([0.175, 0.275, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_R_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, -0.15 - 0.075, 0.075], dtype=np.float64)), 
                                                   np.array([0.125, 0.075, 0.05], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_14000_R_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.35, -0.15 - 0.075, 0.075])
ABB_IRB_14000_R_Env_ID_1_Str.Collision_Object.Scale = 0.015
ABB_IRB_14000_R_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
ABB_IRB_14000_R_Env_ID_1_Str.Collision_Object.Type = 'Sphere'

# ABB IRB 14000 (Left).
#   Environmnet ID 0.
ABB_IRB_14000_L_Env_ID_0_Str = Environment_Str(Name='ABB_IRB_14000_L')
ABB_IRB_14000_L_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15, 0.125 + 0.02], dtype=np.float64)), 
                                                   np.array([0.175, 0.275, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_L_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15, 0.075 + 0.02], dtype=np.float64)), 
                                                   np.array([0.125, 0.225, 0.10], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_14000_L_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
ABB_IRB_14000_L_Env_ID_1_Str = Environment_Str(Name='ABB_IRB_14000_L')
ABB_IRB_14000_L_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15, 0.125 + 0.02], dtype=np.float64)), 
                                                   np.array([0.175, 0.275, 0.25], dtype=np.float64), [1.0, 1.0, 0.0])
ABB_IRB_14000_L_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.35, 0.15 + 0.075, 0.075], dtype=np.float64)), 
                                                   np.array([0.125, 0.075, 0.05], dtype=np.float64), [0.0, 1.0, 0.0])
ABB_IRB_14000_L_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.35, 0.15 + 0.075, 0.075])
ABB_IRB_14000_L_Env_ID_1_Str.Collision_Object.Scale = 0.02
ABB_IRB_14000_L_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
ABB_IRB_14000_L_Env_ID_1_Str.Collision_Object.Type = 'Sphere'

# Epson LS3-B401S.
#   Environmnet ID 0.
EPSON_LS3_B401S_Env_ID_0_Str = Environment_Str(Name='EPSON_LS3_B401S')
EPSON_LS3_B401S_Env_ID_0_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.0, 0.07 + 0.01], dtype=np.float64)), 
                                                   np.array([0.20, 0.30, 0.12], dtype=np.float64), [1.0, 1.0, 0.0])
EPSON_LS3_B401S_Env_ID_0_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.0, 0.05 + 0.01], dtype=np.float64)), 
                                                   np.array([0.15, 0.25, 0.05], dtype=np.float64), [0.0, 1.0, 0.0])
EPSON_LS3_B401S_Env_ID_0_Str.Collision_Object = None
#   Environmnet ID 1.
EPSON_LS3_B401S_Env_ID_1_Str = Environment_Str(Name='EPSON_LS3_B401S')
EPSON_LS3_B401S_Env_ID_1_Str.C.Search = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.0, 0.07 + 0.01], dtype=np.float64)), 
                                                   np.array([0.20, 0.30, 0.12], dtype=np.float64), [1.0, 1.0, 0.0])
EPSON_LS3_B401S_Env_ID_1_Str.C.Target = Cuboid_Str(HTM_Cls(None, np.float64).Translation(np.array([0.25, 0.08, 0.05 + 0.01], dtype=np.float64)), 
                                                   np.array([0.125, 0.075, 0.05], dtype=np.float64), [0.0, 1.0, 0.0])
EPSON_LS3_B401S_Env_ID_1_Str.Collision_Object.T = HTM_Cls(None, np.float64).Translation([0.25, 0.08, 0.05 + 0.01])
EPSON_LS3_B401S_Env_ID_1_Str.Collision_Object.Scale = 0.02
EPSON_LS3_B401S_Env_ID_1_Str.Collision_Object.Color = [0.85, 0.60, 0.60, 0.75]
EPSON_LS3_B401S_Env_ID_1_Str.Collision_Object.Type = 'Sphere'