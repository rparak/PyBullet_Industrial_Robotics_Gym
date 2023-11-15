# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
# Custom Lib.:
#   ../Lib/Primitives/Core
import Lib.Primitives.Core as Primitives
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls
#   ../Lib/Gym/Configuration/Environment
import Lib.Gym.Configuration.Environment

def Add_Wireframe_Cuboid(T: tp.List[tp.List[float]], size: tp.List[float], color: tp.List[float],
                         line_width: float) -> None:
    """
    Description:
        A function to add a wireframe cuboid with defined parameters to the PyBullet environment.

    Args:
        (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the cuboid.
        (2) size [Vector<float> 1x3]: The size of the cuboid.
        (3) color [Vector<float> 1x3]: The color of the cuboid.
                                        Note:
                                            Format: rgba(red, green, blue)
        (4) line_width [float]: The width of the line of the cuboid.

    Returns:
        (1) parameter [Vector<float> 8x3]: Vertices of the object.
    """

    if isinstance(T, (list, np.ndarray)):
        T = HTM_Cls(T, np.float64)

    # Initialization of a primitive object (cuboid).
    Box_Cls = Primitives.Box_Cls([0.0, 0.0, 0.0], size)

    # Store the vertices of the untransformed object.
    vertices = Box_Cls.Vertices.copy()
    
    # Transformation according to the input homogeneous transformation matrix.
    q = T.Get_Rotation('QUATERNION'); p = T.p.all()
    for i, point_i in enumerate(Box_Cls.Vertices):
        vertices[i, :] = q.Rotate(Vector3_Cls(point_i, np.float64)).all() + p

    """
    Initialization of the cuboid edges.
        Note: 
            Lower Base: A {id: 0}, B {id: 1}, C {id: 2}, D {id: 3}
            Upper Base: E {id: 4}, F {id: 5}, G {id: 6}, H {id: 7}
    """
    edges = [[0, 1], [1, 2], [2, 3], [3, 0],
             [4, 5], [5, 6], [6, 7], [7, 4],
             [0, 4], [1, 5], [2, 6], [3, 7]]
    
    # Render the wireframe of the cuboid.
    for _, edges_i in enumerate(edges):
        pb.addUserDebugLine(lineFromXYZ=vertices[edges_i[0]], lineToXYZ=vertices[edges_i[1]], lineColorRGB=color, 
                            lineWidth=line_width)
        
    return vertices

def Get_Environment_Structure(name: str, Env_ID: int) -> Lib.Gym.Configuration.Environment.Configuration_Space_Str:
    """
    Description:
        Obtain the structure of the main parameters of the environment for the defined robotic arm.
    
    Args:
        (1) name [string]: Name of the robotic structure.
        (2) Env_ID [int]: The identification number (ID) of the environment.
                            Note:
                                For more information, see the 
                                script ../Configuration/Environment.py. 

    Returns:
        (1) parameter [Configuration_Space_Str(object)]: Defined structure of the main parameters of the environment.
    """

    try:
        assert Env_ID in [0, 1]

        if Env_ID == 0:
            return {
                'Universal_Robots_UR3': Lib.Gym.Configuration.Environment.Universal_Robots_UR3_Env_ID_0_Str,
                'ABB_IRB_120': Lib.Gym.Configuration.Environment.ABB_IRB_120_Env_ID_0_Str,
                'ABB_IRB_120_L_Ax': Lib.Gym.Configuration.Environment.ABB_IRB_120_L_Ax_Env_ID_0_Str,
                'ABB_IRB_14000_R': Lib.Gym.Configuration.Environment.ABB_IRB_14000_R_Env_ID_0_Str,
                'ABB_IRB_14000_L': Lib.Gym.Configuration.Environment.ABB_IRB_14000_L_Env_ID_0_Str,
                'EPSON_LS3_B401S': Lib.Gym.Configuration.Environment.EPSON_LS3_B401S_Env_ID_0_Str
            }[name]
        else:
            return {
                'Universal_Robots_UR3': Lib.Gym.Configuration.Environment.Universal_Robots_UR3_Env_ID_1_Str,
                'ABB_IRB_120': Lib.Gym.Configuration.Environment.ABB_IRB_120_Env_ID_1_Str,
                'ABB_IRB_120_L_Ax': Lib.Gym.Configuration.Environment.ABB_IRB_120_L_Ax_Env_ID_1_Str,
                'ABB_IRB_14000_R': Lib.Gym.Configuration.Environment.ABB_IRB_14000_R_Env_ID_1_Str,
                'ABB_IRB_14000_L': Lib.Gym.Configuration.Environment.ABB_IRB_14000_L_Env_ID_1_Str,
                'EPSON_LS3_B401S': Lib.Gym.Configuration.Environment.EPSON_LS3_B401S_Env_ID_1_Str
            }[name]
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] An incorrect identification number (ID) for the environment was selected.')