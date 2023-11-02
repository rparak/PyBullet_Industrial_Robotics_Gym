# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
import pybullet_data
# Custom Lib.:
#   ../Lib/Primitives/Core
import Lib.Primitives.Core as Primitives
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls

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