# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
# Custom Lib.: 
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Primitives/Core
import RoLE.Primitives.Core as Primitives
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls
#       ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#   Gym
#       ../Gym/Configuration/Environment
import Gym.Configuration.Environment

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

def Get_Environment_Structure(name: str, Env_ID: int) -> Gym.Configuration.Environment.Environment_Str:
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
        (1) parameter [Environment_Str(object)]: Defined structure of the main parameters of the environment.
    """

    try:
        assert Env_ID in [0, 1]

        if Env_ID == 0:
            return {
                'Universal_Robots_UR3': Gym.Configuration.Environment.Universal_Robots_UR3_Env_ID_0_Str,
                'ABB_IRB_120': Gym.Configuration.Environment.ABB_IRB_120_Env_ID_0_Str,
                'ABB_IRB_120_L_Ax': Gym.Configuration.Environment.ABB_IRB_120_L_Ax_Env_ID_0_Str,
                'ABB_IRB_14000_R': Gym.Configuration.Environment.ABB_IRB_14000_R_Env_ID_0_Str,
                'ABB_IRB_14000_L': Gym.Configuration.Environment.ABB_IRB_14000_L_Env_ID_0_Str,
                'EPSON_LS3_B401S': Gym.Configuration.Environment.EPSON_LS3_B401S_Env_ID_0_Str
            }[name]
        else:
            return {
                'Universal_Robots_UR3': Gym.Configuration.Environment.Universal_Robots_UR3_Env_ID_1_Str,
                'ABB_IRB_120': Gym.Configuration.Environment.ABB_IRB_120_Env_ID_1_Str,
                'ABB_IRB_120_L_Ax': Gym.Configuration.Environment.ABB_IRB_120_L_Ax_Env_ID_1_Str,
                'ABB_IRB_14000_R': Gym.Configuration.Environment.ABB_IRB_14000_R_Env_ID_1_Str,
                'ABB_IRB_14000_L': Gym.Configuration.Environment.ABB_IRB_14000_L_Env_ID_1_Str,
                'EPSON_LS3_B401S': Gym.Configuration.Environment.EPSON_LS3_B401S_Env_ID_1_Str
            }[name]
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] An incorrect identification number (ID) for the environment was selected.')

def Get_Robot_Structure_Theta_Home(name: str, Env_ID: int) -> tp.List[float]:
    """
    Description:
        Get the home absolute joint positions of a specific environment for a defined robotic arm.
    
    Args:
        (1) name [string]: Name of the robotic structure.
        (2) Env_ID [int]: The identification number (ID) of the environment.
                            Note:
                                For more information, see the 
                                script ../Configuration/Environment.py. 

    Returns:
        (1) parameter [Vector<float> 1xn]: The home absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
    """
        
    try:
        assert Env_ID in [0, 1]

        if Env_ID == 0:
            return {
                'Universal_Robots_UR3': Mathematics.Degree_To_Radian(np.array([-71.23467, -98.2801, -90.10467, -81.62488, 89.83034, 19.02965], dtype=np.float64)),
                'ABB_IRB_120': Mathematics.Degree_To_Radian(np.array([0.0, 16.08758, 11.42564, 0.0, 62.48719, 0.0], dtype=np.float64)),
                'ABB_IRB_120_L_Ax': np.append([0.4], Mathematics.Degree_To_Radian(np.array([90.00011, 16.08758, 11.42564, 0.0, 62.48719, 0.0], dtype=np.float64))),
                'ABB_IRB_14000_R': Mathematics.Degree_To_Radian(np.array([86.5391, -131.7427, -52.98846, 60.71872, -162.0104, 116.87471, 18.89269], dtype=np.float64)),
                'ABB_IRB_14000_L': Mathematics.Degree_To_Radian(np.array([-86.74652, -131.67072, 53.04848, 60.71646, -197.85701, 117.01122, -19.01974], dtype=np.float64)),
                'EPSON_LS3_B401S': np.array([Mathematics.Degree_To_Radian(47.16657), Mathematics.Degree_To_Radian(103.77415), 0.0245, Mathematics.Degree_To_Radian(60.94072)], dtype = np.float64)
            }[name]
        else:
            return {
                'Universal_Robots_UR3': Mathematics.Degree_To_Radian(np.array([-92.17399, -97.82379, -90.6596, -81.46499, 89.83809, -1.90976], dtype=np.float64)),
                'ABB_IRB_120': Mathematics.Degree_To_Radian(np.array([-17.04901, 21.08261, 9.79837, -0.00013, 59.11942, -17.04893], dtype=np.float64)),
                'ABB_IRB_120_L_Ax': np.append([0.29171], Mathematics.Degree_To_Radian(np.array([88.97502, 17.83082, 14.63065, 0.0, 57.53893, -1.02511], dtype=np.float64))),
                'ABB_IRB_14000_R': Mathematics.Degree_To_Radian(np.array([97.41684, -129.58253, -67.76966, 58.04787, -176.44575, 120.69353, 30.36447], dtype=np.float64)),
                'ABB_IRB_14000_L': Mathematics.Degree_To_Radian(np.array([-97.64372, -129.49333, 67.80701, 58.03911, -183.48104, 120.87338, -30.5058], dtype=np.float64)),
                'EPSON_LS3_B401S': np.array([Mathematics.Degree_To_Radian(27.96471), Mathematics.Degree_To_Radian(96.37937), 0.0245, Mathematics.Degree_To_Radian(34.34408)], dtype = np.float64)
            }[name]
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] An incorrect identification number (ID) for the environment was selected.')