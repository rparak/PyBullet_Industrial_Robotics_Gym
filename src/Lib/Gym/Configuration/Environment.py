# Notes ...
# Target space denotes where we want the robot to move to. 
# Target Space - C_target
# Search Space - C_search
# Obstacle space 'C_{obstacle}' is a space that the robot can not move to.

# Numpy (Array computing) [pip3 install numpy]tp
import numpy as np
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../

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
    Size: tp.List[tp.List[float]] = field(default_factory=list)

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
    Defined structure of the main parameters of the configuration space 
    for the Universal Robots UR3 robotic arm.
"""
Universal_Robots_UR3_C_Str = Configuration_Space_Str(Name='Universal_Robots_UR3')