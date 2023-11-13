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

# Numpy (Arline computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#  ../Lib/Collider/Utilities
import Lib.Collider.Utilities as Utilities
#  ../Lib/Primitives/Core
import Lib.Primitives.Core as Primitives
#  ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls, Get_Matrix_Identity
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

"""
Description:
    Some useful references about the field of object collisions:
        1. Real-Time Collision Detection, Christer Ericson
        2. Collision Detection in Interactive 3D Environments, Gino van den Bergen
        3. Game Physics Cookbook, Gabor Szauer
        4. Essential Mathematics for Game and Interactive Applications: A Programmer's Guide, James M.VanVerth and Lars M. Bishop

    The main functions of the library are:
        A. 3D Shape Intersection
            - AABB <-> AABB
            - OBB <-> OBB
            - OBB <-> AABB
        B. 3D Line Intersections (Raycast)
            - Line Segment (Ray) <-> AABB
            - Line Segment (Ray) <-> OBB
        C. 3D Point Tests
            - Point Inside AABB
            - Point Inside OBB
"""

"""
Description:
    Initialization of constants.
"""
# Type of collider as a numeric value.
CONST_TYPE_AABB = 0
CONST_TYPE_OBB  = 1

class AABB_Cls(object):
    """
    Description:
        A specific class for working with Axis-aligned Bounding Boxes (AABBs).

        The initialization of the three-dimensional box can be found in the script ../Primitives.py. Other tasks will be performed 
        by the main class AABB_Cls(object).

        Note:
            An Axis-aligned Bounding Boxes (AABBs) is a rectangular six-sided box (in 3D, four-sided in 2D).

    Initialization of the Class:
        Args:
            (1) Box [Primitives.Box_Cls(object)]: A specific class for working with boxes.

        Example:
            Initialization:
                # Assignment of the variables.
                Box = Primitives.Box_Cls([0.0, 0.0, 0.0], 
                                         [1.0, 1.0, 1.0])

                # Initialization of the class.
                Cls = OBB_Cls(Box)

            Features:
                # Properties of the class.
                Cls.Size; Cls.Vertices
                ...
                Cls.T

                # Functions of the class.
                Cls.Transformation(T: HTM_Cls(object))
                ...
                Cls.Overlap(object: [AABB_Cls(object), OBB_Cls(object)])
    """
        
    # Create a global data type for the class.
    AABB_Cls_dtype = tp.TypeVar('AABB_Cls_dtype')
    OBB_Cls_dtype = tp.TypeVar('OBB_Cls_dtype')

    def __init__(self, Box: Primitives.Box_Cls) -> None:
        try:
            assert isinstance(Box, Primitives.Box_Cls)

            # << PRIVATE >> #
            # Get important information from the input object.
            #   Main class object.
            self.__Box = Box
            #   Properties of the object.
            self.__vertices = self.__Box.Vertices.copy()
            self.__size     = self.__Box.Size
            self.__T        = self.__Box.T
            # Create the identity matrix in rotation matrix form.
            self.__R_Identity = Get_Matrix_Identity(3)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of class input parameters.')

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

        return self.__Box.Origin
    
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
    def Type(self) -> int:
        """
        Description:
            Get a specific object type (ID: identification number).

        Returns:
            (1) parameter [int]: Object type.
        """

        return 0

    @property
    def T(self) -> HTM_Cls:
        """
        Description:
            Get the object's transformation with zero rotation.

        Returns:
            (1) parameter [HTM_Cls(object) -> Matrix<float> 4x4]: Homogeneous transformation matrix 
                                                                  of the object.
        """

        return self.__T

    def Transformation(self, T: HTM_Cls) -> None:
        """
        Description:
            Axis-aligned Bounding Box (AABB) transformation according to the input homogeneous transformation matrix.

            Note:
                AABB is not defined for orientation, so we only use the translation part for transformation.

        Args:
            (1) T [HTM_Cls(object) -> Matrix<float> 4x4]: Desired homogeneous transformation matrix (HTM) for object 
                                                          transformation.
        """

        try:   
            assert isinstance(T, HTM_Cls)

            # Store the input object in a class variable.
            self.__T = T

            try:
                assert (self.__T.R == self.__R_Identity).all()

                p = self.__T.p.all()
                for i, verts_i in enumerate(self.__Box.Vertices):
                    self.__vertices[i, :] = verts_i + p

            except AssertionError as error:
                print(f'[ERROR] Information: {error}')
                print('[ERROR] The rotation part of the input homogeneous transformation matrix is not in the identity form.')

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

    def Is_Point_Inside(self, point: Primitives.Point_Cls) -> bool:
        """
        Description:
            A function to determine if a given point is inside a geometric object. More precisely inside the AABB.

        Args:
            (1) point [Primitives.Point_Cls(object)]: A three-dimensional point to check whether it is inside or not.

        Returns:
            (1) parameter [bool]: If the result is true, the point is inside, otherwise it is not.
        """
                
        try:
            assert isinstance(point, Primitives.Point_Cls)

            # Get the minimum and maximum X, Y, Z values of the input vertices.
            (aabb_min, aabb_max) = Utilities.Get_Min_Max(self.__vertices)

            # Check if the minimum and maximum X, Y, Z values of the bounding box overlap 
            # with the input point.
            for _, (aabb_min_i, aabb_max_i, c_i) in enumerate(zip(aabb_min, aabb_max, point.Centroid)):
                if aabb_min_i > c_i:
                    return False

                if aabb_max_i < c_i:
                    return False

            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')
            
    def Raycast(self, line_segment: Primitives.Line_Segment_Cls) -> tp.Tuple[bool, tp.Union[None, tp.List[float]]]:
        """
        Description:
            A function to check if a line segment intersects with 3D primitive object (AABB).

            Note:
                If it intersects, we can find one or two points where it specifically intersects the object (AABB).

        Args:
            (1) line_segment [Primitives.Line_Segment_Cls(object)]: A specific class for working with a line segment (ray).

        Returns:
            (1) parameter [bool]: The result of whether or not the input line segment (ray) intersects with the class object.
                                  Note:
                                    The value is "True" if the line segment intersects with the object, and "False" if it does not.
            (2) parameter [None or Vector<float> 2x3]: The points where the line segment intersects the box.
                                                       Note:
                                                        The value is "None" if there is no intersection.
        """

        try:
            assert isinstance(line_segment, Primitives.Line_Segment_Cls)

            # Get the minimum and maximum X, Y, Z values of the input vertices.
            (aabb_min, aabb_max) = Utilities.Get_Min_Max(self.__vertices)

            for i, (aabb_min_i, aabb_max_i, ls_c_i, ls_dir_i) in enumerate(zip(aabb_min, aabb_max, 
                                                                               line_segment.Centroid, line_segment.Direction)):
                # Check that the line segment direction {line_dir_i} is equal to zero.
                if Utilities.CMP(ls_dir_i, 0.0, Mathematics.CONST_EPS_32) != True:
                    ls_dir_i_tmp = ls_dir_i
                else:
                    # Set the variable to a small value to avoid division by ze ro.
                    ls_dir_i_tmp = 1e-5

                t_0_tmp = (aabb_min_i - ls_c_i) / ls_dir_i_tmp
                t_1_tmp = (aabb_max_i - ls_c_i) / ls_dir_i_tmp

                # To complete the method, we need to find the largest minimum {t_min} and the smallest 
                # maximum {t_max}.
                if i == 0:
                    t_min = Mathematics.Min([t_0_tmp, t_1_tmp])[1]
                    t_max = Mathematics.Max([t_0_tmp, t_1_tmp])[1]
                else:
                    t_min = Mathematics.Max([t_min, Mathematics.Min([t_0_tmp, t_1_tmp])[1]])[1]
                    t_max = Mathematics.Min([t_max, Mathematics.Max([t_0_tmp, t_1_tmp])[1]])[1]

            """
            Description:
                If the following condition holds, there is no intersection.
            """

            if t_min > t_max:
                return (False, None)
            
            """
            Description:
                Otherwise, there is an intersection.
            """
            return (True, Utilities.Get_Points_of_Intersection(t_min, t_max, line_segment))

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

    def Overlap(self, object: tp.Union[AABB_Cls_dtype, OBB_Cls_dtype]) -> bool:
        """
        Description:
            A function to check if two 3D primitives overlap (intersect) or not.

            In our case, these are 3D primitives:
                1\ Axis-aligned Bounding Box (AABB)
                2\ Oriented Bounding Box (OBB)

            The function can solve both AABB <-> AABB and AABB <-> OBB intersections.

            Note:
                This function can be simply called as collision check.

        Args:
            (1) object [AABB_Cls(object) or OBB_Cls(object)]: A specific class for working with OBB or AABB.

        Returns:
            (1) parameter [bool]: The result of whether or not the input object overlaps with the class object.
                                  Note:
                                    The value is "True" if the objects overlap and "False" if they do not.
        """
                
        try:
            assert isinstance(object, (AABB_Cls, OBB_Cls))

            if object.Type == CONST_TYPE_AABB:
                """
                Method: AABB <-> AABB
                    Testing whether the two AABBs overlap.
                """ 

                # Get the minimum and maximum X, Y, Z values of the input vertices.
                #   1\ Class Object.
                (aabb_min, aabb_max) = Utilities.Get_Min_Max(self.__vertices)
                #   2\ Input Object.
                (object_min, object_max) = Utilities.Get_Min_Max(object.Vertices)

                # Check if the minimum and maximum X, Y, Z values of the bounding boxes overlap.
                for _, (aabb_min_i, aabb_max_i, obj_min_i, obj_max_i) in enumerate(zip(aabb_min, aabb_max,
                                                                                       object_min, object_max)):
                    if aabb_min_i > obj_max_i:
                        return False

                    if aabb_max_i < obj_min_i:
                        return False
                
                return True
            
            elif object.Type == CONST_TYPE_OBB:
                """
                Method: AABB <-> OBB
                    Testing whether AABB and OBB overlap.
                """ 
                                
                return object.Overlap(self)
                
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

class OBB_Cls(object):
    """
    Description:
        A specific class for working with Oriented Bounding Boxes (OBBs).

        The initialization of the three-dimensional box can be found in the script ../Primitives.py. Other tasks will be performed 
        by the main class OBB_Cls(object).

        Note:
            An oriented bounding boxes (OBBs) is a rectangular block, similar to an AABB, but with arbitrary 
            orientation.

    Initialization of the Class:
        Args:
            (1) Box [Primitives.Box_Cls(object)]: A specific class for working with boxes.

        Example:
            Initialization:
                # Assignment of the variables.
                Box = Primitives.Box_Cls([0.0, 0.0, 0.0], 
                                         [1.0, 1.0, 1.0])

                # Initialization of the class.
                Cls = OBB_Cls(Box)

            Features:
                # Properties of the class.
                Cls.Size; Cls.Vertices
                ...
                Cls.T

                # Functions of the class.
                Cls.Transformation(T: HTM_Cls(object))
                ...
                Cls.Overlap(object: [AABB_Cls(object), OBB_Cls(object)])
    """
       
    # Create a global data type for the class.
    AABB_Cls_dtype = tp.TypeVar('AABB_Cls_dtype')
    OBB_Cls_dtype = tp.TypeVar('OBB_Cls_dtype')

    def __init__(self, Box: Primitives.Box_Cls) -> None:
        try:
            assert isinstance(Box, Primitives.Box_Cls)

            # << PRIVATE >> #
            # Get important information from the input object.
            #   Main class object.
            self.__Box = Box
            #   Properties of the object.
            self.__vertices = self.__Box.Vertices.copy()
            self.__size     = self.__Box.Size
            self.__T        = self.__Box.T

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of class input parameters.')

    @property
    def Size(self) -> tp.List[float]:
        """
        Description:
            Get the size of the box in the defined space.

        Returns:
            (1) parameter [float]: Box size (X, Y, Z).
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

        return self.__Box.Origin
    
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
    def Type(self) -> int:
        """
        Description:
            Get a specific object type (ID: identification number).

        Returns:
            (1) parameter [int]: Object type.
        """
        
        return 1

    @property
    def T(self) -> HTM_Cls:
        """
        Description:
            Get the object's transformation with zero rotation.

        Returns:
            (1) parameter [HTM_Cls(object) -> Matrix<float> 4x4]: Homogeneous transformation matrix 
                                                                  of the object.
        """

        return self.__T

    def Transformation(self, T: HTM_Cls) -> None:
        """
        Description:
            Oriented Bounding Box (OBB) transformation according to the input homogeneous transformation matrix.

        Args:
            (1) T [HTM_Cls(object) -> Matrix<float> 4x4]: Desired homogeneous transformation matrix (HTM) for object 
                                                          transformation.
        """

        try:   
            assert isinstance(T, HTM_Cls)

            # Store the input object in a class variable.
            self.__T = T

            q = self.__T.Get_Rotation('QUATERNION'); p = self.__T.p.all()
            for i, point_i in enumerate(self.__Box.Vertices):
                self.__vertices[i, :] = q.Rotate(Vector3_Cls(point_i, np.float64)).all() + p

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

    def Is_Point_Inside(self, point: Primitives.Point_Cls) -> bool:
        """
        Description:
            A function to determine if a given point is inside a geometric object. More precisely inside the OBB.

        Args:
            (1) point [Primitives.Point_Cls(object)]: A three-dimensional point to check whether it is inside or not.

        Returns:
            (1) parameter [bool]: If the result is true, the point is inside, otherwise it is not.
        """

        try:
            assert isinstance(point, Primitives.Point_Cls)

            # Each axis of the bounding box (OBB) rotation frame as a vector.
            axis = np.zeros((3, Primitives.CONST_DIMENSION), dtype = np.float64)
            #    X, Y and Z axis (orientation) the bounding box (OBB).
            axis[0] = self.__T.R[:, 0]; axis[1] = self.__T.R[:, 1]
            axis[2] = self.__T.R[:, 2]

            # Direction vector from the centroid of the object (OBB) to the point.
            v = self.__T.p.all() - point.Centroid

            # A point P is inside the object (OBB) only if all the following conditions 
            # are satisfied. Otherwise it is outside the object (OBB).
            for _, (ax_i, size_i) in enumerate(zip(axis, self.__size * 0.5)):
                if np.absolute(Vector3_Cls(ax_i, np.float64).Dot(v)) >= size_i:
                    return False

            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

    def Raycast(self, line_segment: Primitives.Line_Segment_Cls) -> tp.Tuple[bool, tp.Union[None, tp.List[float]]]:
        """
        Description:
            A function to check if a line segment intersects with 3D primitive object (OBB).

            Note:
                If it intersects, we can find one or two points where it specifically intersects the object (OBB).

        Args:
            (1) line_segment [Primitives.Line_Segment_Cls(object)]: A specific class for working with a line segment (ray).

        Returns:
            (1) parameter [bool]: The result of whether or not the input line segment (ray) intersects with the class object.
                                  Note:
                                    The value is "True" if the line segment intersects with the object, and "False" if it does not.
            (2) parameter [None or Vector<float> 2x3]: The points where the line segment intersects the box.
                                                       Note:
                                                        The value is "None" if there is no intersection.
        """
                
        try:
            assert isinstance(line_segment, Primitives.Line_Segment_Cls)
            
            # Each axis of the bounding box (OBB) rotation frame as a vector.
            axis = np.zeros((3, Primitives.CONST_DIMENSION), dtype = np.float64)
            #    X, Y and Z axis (orientation) the bounding box (OBB).
            axis[0] = self.__T.R[:, 0]; axis[1] = self.__T.R[:, 1]
            axis[2] = self.__T.R[:, 2]

            # Direction vector from the centroid of the object (OBB) to the segment centroid.
            v = self.__T.p.all() - line_segment.Centroid

            for i, (ax_i, size_i) in enumerate(zip(axis, self.__size * 0.5)):
                # Projection:
                #   1\ {v} onto each axis of the bounding box (OBB).
                e_tmp = Vector3_Cls(ax_i, np.float64).Dot(v)
                #   2\ {direction of the line segment} onto each axis of the bounding box (OBB).
                f_tmp = Vector3_Cls(ax_i, np.float64).Dot(line_segment.Direction)

                # Check that the projection {f} is equal to zero.
                if Utilities.CMP(f_tmp, 0.0, Mathematics.CONST_EPS_32) == True:
                    # If the segment (ray) is parallel to the plate under test and the origin of the segment is not inside 
                    # the plate, there is no intersection.
                    if (-e_tmp - size_i > 0) or (-e_tmp + size_i < 0):
                        return (False, None)
                    
                    # Set the variable to a small value to avoid division by zero.
                    f_tmp = 1e-5

                # Intersection with the planes (right, left).
                t_0_tmp = (e_tmp - size_i) / f_tmp
                t_1_tmp = (e_tmp + size_i) / f_tmp

                # To complete the method, we need to find the largest minimum {t_min} and the smallest 
                # maximum {t_max}.
                if i == 0:
                    t_min = Mathematics.Min([t_0_tmp, t_1_tmp])[1]
                    t_max = Mathematics.Max([t_0_tmp, t_1_tmp])[1]
                else:
                    t_min = Mathematics.Max([t_min, Mathematics.Min([t_0_tmp, t_1_tmp])[1]])[1]
                    t_max = Mathematics.Min([t_max, Mathematics.Max([t_0_tmp, t_1_tmp])[1]])[1]

            """
            Description:
                If the following condition holds, there is no intersection.
            """

            if t_min > t_max:
                return (False, None)
            
            """
            Description:
                Otherwise, there is an intersection.
            """
            return (True, Utilities.Get_Points_of_Intersection(t_min, t_max, line_segment))
    
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')

    @staticmethod
    def __Get_Interal(axis: tp.List[float], verts: tp.List[tp.List[float]]) -> tp.Tuple[float, float]:
        """
        Description:
            The function to get the interval of the shapes specified by the axis.

        Args:
            (1) axis [Vector<float> 1x3]: The individual axes of the bounding box (OBB, AABB) generated 
                                          from the Overlap function.
            (2) verts [Vector<float> 8x3]: The vertices of the object.

        Returns:
            (1, 2) parameter [float]: Minimum and maximum projection in an interval structure.
        """
                
        out_min = Vector3_Cls(axis, np.float64).Dot(verts[0])
        out_max = out_min.copy()

        # Projection of individual vertices on the specified axes.
        for _, verts_i in enumerate(verts):
            # Projection of the axis onto the individual vertices 
            # of the bounding box (OBB, AABB).
            projection = Vector3_Cls(axis, np.float64).Dot(verts_i)
            
            # Store the minimum and maximum projection in an interval structure.
            out_min = projection if projection < out_min else out_min
            out_max = projection if projection > out_max else out_max

        return (out_min, out_max)
    
    def Overlap(self, object: tp.Union[AABB_Cls_dtype, OBB_Cls_dtype]) -> bool:
        """
        Description:
            A function to check if two 3D primitives overlap (intersect) or not.

            In our case, these are 3D primitives:
                1\ Axis-aligned Bounding Box (AABB)
                2\ Oriented Bounding Box (OBB)

            The function can solve both OBB <-> OBB and OBB <-> AABB intersections.

            Note:
                This function can be simply called as collision check.

        Args:
            (1) object [AABB_Cls(object) or OBB_Cls(object)]: A specific class for working with OBB or AABB.

        Returns:
            (1) parameter [bool]: The result of whether or not the input object overlaps with the class object.
                                  Note:
                                    The value is "True" if the objects overlap and "False" if they do not.
        """
                
        try:
            assert isinstance(object, (AABB_Cls, OBB_Cls))

            """
            Method: OBB <-> OBB, OBB <-> AABB
                Testing whether two OBBs or AABB and OBB overlap.

                Note:
                    We can use the same algorithm because we have information about 
                    the transformation of both objects.
            """ 
            
            # Each axis of the bounding box (OBB, AABB) rotation frame as a vector. There are 15 potential axes to test.
            axis = np.zeros((15, Primitives.CONST_DIMENSION), dtype = np.float64)
            #    X, Y and Z axis (orientation) of the bounding box (OBB or AABB): Input Object.
            axis[0] = object.T.R[:, 0]; axis[1] = object.T.R[:, 1]
            axis[2] = object.T.R[:, 2]
            #    X, Y and Z axis (orientation) of the bounding box (OBB): Class Object.
            axis[3] = self.__T.R[:, 0]; axis[4] = self.__T.R[:, 1]
            axis[5] = self.__T.R[:, 2]

            #   The last nine axes are the cross product of all the previous axes calculated above.
            for i in range(0, Primitives.CONST_DIMENSION):
                axis[6 + i * 3] = Vector3_Cls(axis[i], np.float64).Cross(axis[3]).all()
                axis[7 + i * 3] = Vector3_Cls(axis[i], np.float64).Cross(axis[4]).all()
                axis[8 + i * 3] = Vector3_Cls(axis[i], np.float64).Cross(axis[5]).all()

            # Check if the minimum and maximum projection in the interval structure 
            # of the shapes overlap.
            for _, ax_i in enumerate(axis):
                # A function to get the interval of both shapes on a given axis.
                (cls_min_i, cls_max_i) = self.__Get_Interal(ax_i, self.__vertices)
                (obj_min_i, obj_max_i) = self.__Get_Interal(ax_i, object.Vertices)

                if obj_min_i > cls_max_i:
                    return False
                
                if obj_max_i < cls_min_i:
                    return False

            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of function input parameters.')
