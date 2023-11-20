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
File Name: Core.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls

"""
Description:
    Initialization of constants.
"""
# The calculation will be performed in three-dimensional (3D) space.
CONST_DIMENSION = 3
# Shape (Box):
#   Vertices: 8; Space: 3D;
CONST_BOX_SHAPE = (8, CONST_DIMENSION)

class Point_Cls(object):
    """
    Description:
        A specific class for working with three-dimensional point as a primitive object.

        Note:
            A point is a location in space whose position is described by the coordinates of a coordinate 
            system given by a reference point, called the centroid (or origin), and several coordinate axes.

            Equation:
                p = (x, y, z)

    Initialization of the Class:
        Args:
            (1) centroid [Vector<float> 1x3]: Center position (X, Y, Z) of the point. 

        Example:
            Initialization:
                # Assignment of the variables.
                p   = [0.0, 0.0, 0.0]
                p_n = [1.0, 0.0, 0.0]

                # Initialization of the class.
                Cls = Point_Cls(p)

            Features:
                # Properties of the class.
                Cls.Centroid

                # Functions of the class.
                Cls.Transformation(p_n)
    """

    def __init__(self, centroid: tp.List[float] = [0.0] * CONST_DIMENSION) -> None:
        # << PRIVATE >> #
        self.__centroid = np.array(centroid, np.float64)

    @property
    def Centroid(self) -> tp.List[float]:
        """
        Description:
            Get the center position (centroid) of the point.

        Returns:
            (1) parameter [Vector<float> 1x3]: Center position (X, Y, Z) of the point. 
        """

        return self.__centroid

    def Transformation(self, centroid: tp.List[float]) -> None:
        """
        Description:
            Transformation of point position in X, Y, Z axes.

        Args:
            (1) centroid [Vector<float> 1x3]: The desired center position (X, Y, Z) of the point.
        """

        self.__centroid = np.array(centroid, np.float64)

class Line_Segment_Cls(object):
    """
    Description:
        A specific class for working with a line segment (ray) as a primitive object.

        Note:
            A line L can be defined as the set of points expressible as the linear combination of two arbitrary 
            but distinct points {a: start point} and {b: end point}. The line segment connecting {a} and {b} is the finite 
            part of the line passing through {a} and {b}, given by the constraint t such that it lies in the 
            interval 0.0 <= t <= 1.0.

            Equation:
                L(t) = (1 - t)*a + t*b, where t can take values 0.0 <= t <= 1.0.

    Initialization of the Class:
        Args:
            (1) a [Vector<float> 1x3]: Start point of the line segment (ray).
            (2) b [Vector<float> 1x3]: End point of the line segment (ray).

        Example:
            Initialization:
                # Assignment of the variables.
                a = [0.0, 0.0, 0.0]
                b = [0.0, 0.0, 5.0]

                # Initialization of the class.
                Cls = Line_Cls(a, b)

            Features:
                # Properties of the class.
                Cls.a; Cls.b
                Cls.Direction; Cls.Centroid
    """
        
    def __init__(self, a: tp.List[float] = [0.0] * CONST_DIMENSION, b: tp.List[float] = [0.0] * CONST_DIMENSION) -> None:
        # << PRIVATE >> #
        self.__a = a; self.__b = b

        # To obtain the centroid, we use the formula:
        #   (1 - t)*a + t*b = (1 - 0.5)*a + 0.5 * b
        #   Note:
        #       t is equal to 0.5, which is half of the line segment.
        self.__centroid = np.array([0.5 * self.__a[0] + 0.5 * self.__b[0], 
                                    0.5 * self.__a[1] + 0.5 * self.__b[1], 
                                    0.5 * self.__a[2] + 0.5 * self.__b[2]])
                                    
    @property
    def Direction(self) -> tp.List[float]:
        """
        Description:
            Get the direction of the line segment.

        Returns:
            (1) parameter [float]: Direction of the line segment.
        """
                
        return self.__a - self.__b

    @property
    def a(self) -> float:
        """
        Description:
            Get the start point of the line segment.

        Returns:
            (1) parameter [float]: Start point {a} of the line segment.
        """
                
        return self.__a

    @property
    def b(self) -> float:
        """
        Description:
            Get the end point of the line segment.

        Returns:
            (1) parameter [float]: End point {b} of the line segment.
        """
                
        return self.__b

    @property
    def Centroid(self) -> tp.List[float]:
        """
        Description:
            Get the center position (centroid) of the line segment.

        Returns:
            (1) parameter [Vector<float> 1x3]: Center position (X, Y, Z) of the line segment. 
        """
                
        return self.__centroid

class Box_Cls(object):
    """
    Description:
        A specific class for working with box (or cuboid) as a primitive object.

        Note:
            The box (or cuboid) is the 3D version of a rectangle. We can define a three-dimensional box by the origin 
            and a size.

    Initialization of the Class:
        Args:
            (1) origin [Vector<float> 1x3]: The origin of the object.
            (2) size [Vector<float> 1x3]: The size of the object.

        Example:
            Initialization:
                # Assignment of the variables.
                origin = [0.0, 0.0, 0.0]
                size   = [1.0, 1.0, 1.0]

                # Initialization of the class.
                Cls = Box_Cls(origin, size)

            Features:
                # Properties of the class.
                Cls.Size; Cls.T; Cls.Vertices
    """
        
    def __init__(self, origin: tp.List[float] = [0.0] * CONST_DIMENSION, size: tp.List[float] = [0.0] * CONST_DIMENSION) -> None:
        # << PRIVATE >> #
        self.__size = np.array(size, np.float64)

        # The origin of the object.
        self.__origin = np.array(origin, np.float64)

        # Calculate the object's centroid from the object's origin.
        self.__centroid = np.array([0.0] * CONST_DIMENSION, np.float64) - self.__origin

        # Convert the initial object sizes to a transformation matrix.
        self.__T_Size = HTM_Cls(None, np.float64).Scale(self.__size)

        # Calculate the vertices of the box defined by the input parameters of the class.
        self.__vertices = np.zeros(CONST_BOX_SHAPE, dtype=np.float64)
        for i, verts_i in enumerate(self.__Get_Init_Vertices()):
            self.__vertices[i, :] = (self.__T_Size.all() @ np.append(verts_i, 1.0).tolist())[0:3] - self.__origin

    @staticmethod
    def __Get_Init_Vertices() -> tp.List[tp.List[float]]:
        """
        Description:
            Helper function to get the initial vertices of the object.

            Note: 
                Lower Base: A {id: 0}, B {id: 1}, C {id: 2}, D {id: 3}
                Upper Base: E {id: 4}, F {id: 5}, G {id: 6}, H {id: 7}

        Returns:
            (1) parameter [Vector<float> 8x3]: Vertices of the object.
        """
 
        return np.array([[0.5, -0.5, -0.5], [0.5, 0.5, -0.5], [-0.5, 0.5, -0.5], [-0.5, -0.5, -0.5],
                         [0.5, -0.5,  0.5], [0.5, 0.5,  0.5], [-0.5, 0.5,  0.5], [-0.5, -0.5,  0.5]], dtype=np.float64)

    @property
    def Size(self) -> tp.List[float]:
        """
        Description:
            Get the size of the box in the defined space.

        Returns:
            (1) parameter [Vector<float> 1x3]: Box size (X, Y, Z).
        """
                
        return self.__size

    @property
    def Origin(self) -> tp.List[float]:
        """
        Description:
            Get the origin of the box.

        Returns:
            (1) parameter [Vector<float> 1x3]: Box origin (X, Y, Z).
        """

        return self.__origin
    
    @property
    def Vertices(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the vertices of the object.

        Returns:
            (1) parameter [Vector<float> 8x3]: Vertices of the object.
        """

        return self.__vertices

    @property
    def Faces(self) -> tp.List[tp.List[tp.List[float]]]:
        """
        Description:
            Get the faces of the object.

        Returns:
            (1) parameter [Vector<float> 6x4x3]: Faces of the object.
        """

        return np.array([[self.__vertices[0], self.__vertices[1], self.__vertices[2], self.__vertices[3]],
                         [self.__vertices[4], self.__vertices[5], self.__vertices[6], self.__vertices[7]],
                         [self.__vertices[3], self.__vertices[0], self.__vertices[4], self.__vertices[7]],
                         [self.__vertices[2], self.__vertices[1], self.__vertices[5], self.__vertices[6]],
                         [self.__vertices[0], self.__vertices[1], self.__vertices[5], self.__vertices[4]],
                         [self.__vertices[3], self.__vertices[2], self.__vertices[6], self.__vertices[7]]], dtype=np.float64)
        
    @property
    def T(self) -> HTM_Cls:
        """
        Description:
            Get the object's transformation with zero rotation.

        Returns:
            (1) parameter [HTM_Cls(object) -> Matrix<float> 4x4]: Homogeneous transformation matrix 
                                                                  of the object.
        """

        return HTM_Cls(None, np.float64).Translation(self.__centroid)