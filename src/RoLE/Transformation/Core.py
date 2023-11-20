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
File Name: Transformation.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
# ../Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics

def Is_Matrix_Square(M: tp.List[tp.List[float]]):
    """
    Description:
        A function to check if the shape of the matrix is square.

    Returns:
        (1) parameter [bool]: Information about whether the shape of the matrix is square.
    """

    return M.shape[0] == M.shape[1]
    
def Get_Matrix_Identity(n: int) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the identity matrix.

        Note:
            The identity array is a square array with ones on the main diagonal.
    Args:
        (1) n [int]: Number of rows/columns in the output identity matrix nxn.

    Returns:
        (1) parameter [Matrix<float> nxn]: Identity matrix.
    """

    M = np.zeros((n, n), dtype=np.float64)
    for i in range(n):
        M[i, i] = 1.0

    return M

def Get_Matrix_Tranpose(M: tp.List[tp.List[float]]) -> tp.List[tp.List[float]]:
        """
        Description:
            Get a homogeneous transformation matrix {T} with transposed axes.

        Args:
            (1) M [Matrix<float> mxn]: 

        Returns:
            (1) parameter [Matrix<float> mxn]: Transposed matrix.
        """

        M_out = np.zeros((M.shape[1], M.shape[0]), dtype=np.float64)
        for i, M_out_i in enumerate(M):
            for j, M_out_ij in enumerate(M_out_i):
                M_out[j][i] = M_out_ij

        return M_out

def Get_Matrix_Diagonal(M: tp.List[tp.List[float]]) -> tp.List[float]:
    """
    Description:
        Get the vector of elements of the diagonal of the homogeneous transformation 
        matrix.

    Args:
        (1) M [Matrix<float> nxn]: Homogeneous transformation matrix.
        
    Returns:
        (1) parameter [Vector<float> 1x4]: Vector of elements of the diagonal of the input matrix.
    """

    v_diag = np.zeros(M.shape[0], dtype=np.float64)
    for j, M_i in enumerate(M):
        v_diag[j] = M_i[j]

    return v_diag

def Get_Matrix_Trace(M: tp.List[tp.List[float]]) -> float:
    """
    Description:
        Get the sum of the elements on the main diagonal (upper left to lower 
        right) of the homogeneous transformation matrix.

        Equation:
            tr(M) = sum(M_{ii})

    Args:
        (1) M [Matrix<float> nxn]: Homogeneous transformation matrix.

    Returns:
        (1) parameter [float]: Trace of the nxn square matrix.
    """

    tr_T = 0.0
    for j, M_i in enumerate(M):
        tr_T += M_i[j]

    return tr_T

def Get_Translation_Matrix(ax: str, theta: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the homogeneous transformation matrix (translation) defined by single parameter.
        
    Args:
        (1) ax [string]: Axis name.
        (2) theta [float]: Desired absolute joint position in meters.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix moved in a specific translation axis.
    """

    return {
        'X': lambda x: Homogeneous_Transformation_Matrix_Cls(Get_Matrix_Identity(4), np.float64).Translation([  x, 0.0, 0.0]),
        'Y': lambda x: Homogeneous_Transformation_Matrix_Cls(Get_Matrix_Identity(4), np.float64).Translation([0.0,   x, 0.0]),
        'Z': lambda x: Homogeneous_Transformation_Matrix_Cls(Get_Matrix_Identity(4), np.float64).Translation([0.0, 0.0,   x])
    }[ax](theta)

def __Get_Rotation_Matrix_X(theta: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        A rotation matrix that rotates the vectors by an angle {theta} about the {x}-axis 
        using a right-hand rule that encodes the alternation of signs.

        Note:
            R_{x}(theta) = [1.0,        0.0,         0.0, 0.0]
                           [0.0, cos(theta), -sin(theta), 0.0]
                           [0.0, sin(theta),  cos(theta), 0.0]
                           [0.0,        0.0,         0.0, 1.0]

    Args:
        (1) theta [float]: Required angle of rotation in radians.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix around {x}-axis.
    """

    # Abbreviations for individual angle functions.
    c_th = np.cos(theta); s_th = np.sin(theta)

    return Homogeneous_Transformation_Matrix_Cls([[ 1.0,  0.0,         0.0, 0.0],
                                                  [ 0.0, c_th, (-1.0)*s_th, 0.0],
                                                  [ 0.0, s_th,        c_th, 0.0],
                                                  [ 0.0,  0.0,         0.0, 1.0]], np.float64)

def __Get_Rotation_Matrix_Y(theta: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        A rotation matrix that rotates the vectors by an angle {theta} about the {y}-axis 
        using a right-hand rule that encodes the alternation of signs.

        Note:
            R_{y}(theta) = [ cos(theta), 0.0,  sin(theta), 0.0]
                           [        0.0, 1.0,         0.0, 0.0]
                           [-sin(theta), 0.0,  cos(theta), 0.0]
                           [        0.0, 0.0,         0.0, 1.0]

    Args:
        (1) theta [float]: Required angle of rotation in radians.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix around {y}-axis.
    """
    
    # Abbreviations for individual angle functions.
    c_th = np.cos(theta); s_th = np.sin(theta)

    return Homogeneous_Transformation_Matrix_Cls([[        c_th, 0.0, s_th, 0.0],
                                                  [         0.0, 1.0,  0.0, 0.0],
                                                  [ (-1.0)*s_th, 0.0, c_th, 0.0],
                                                  [         0.0, 0.0,  0.0, 1.0]], np.float64)

def __Get_Rotation_Matrix_Z(theta: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        A rotation matrix that rotates the vectors by an angle {theta} about the {z}-axis 
        using a right-hand rule that encodes the alternation of signs.

        Note:
            R_{z}(theta) = [cos(theta), -sin(theta), 0.0, 0.0]
                           [sin(theta),  cos(theta), 0.0, 0.0]
                           [       0.0,        0.0,  1.0, 0.0]
                           [       0.0,        0.0,  0.0, 1.0]

    Args:
        (1) theta [float]: Required angle of rotation in radians.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix around {z}-axis.
    """

    # Abbreviations for individual angle functions.
    c_th = np.cos(theta); s_th = np.sin(theta)

    return Homogeneous_Transformation_Matrix_Cls([[ c_th, (-1.0)*s_th, 0.0, 0.0],
                                                  [ s_th,        c_th, 0.0, 0.0],
                                                  [ 0.0,          0.0, 1.0, 0.0],
                                                  [  0.0,         0.0, 0.0, 1.0]], np.float64)

def Get_Rotation_Matrix(ax: str, theta: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the homogeneous transformation matrix around a specific rotation axis.
        
    Args:
        (1) ax [string]: Axis name.
        (2) theta [float]: Desired absolute joint position in radians.
        
    Returns
        (1) parameter [INT]: [Matrix<float> 4x4]: Homogeneous transformation matrix around a specific rotation axis.
    """

    return {
        'X': lambda x: __Get_Rotation_Matrix_X(x),
        'Y': lambda x: __Get_Rotation_Matrix_Y(x),
        'Z': lambda x: __Get_Rotation_Matrix_Z(x),
    }[ax](theta)

def Get_Matrix_From_Euler_Method_Standard(theta: tp.List[float], axes_sequence_cfg: str) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the homogeneous transformation matrix from Euler angles (also known as Tait-Bryan / Cardan angles or yaw, pitch, and roll angles) in 
        a specified sequence of axes.

        There are six choices of rotation axes:
            intrinsic rotation (x - y' - z'') -> extrinsic rotation (z - y - x)
            ...
            intrinsic rotation (z - y' - x'') -> extrinsic rotation (x - y - z)

    Args:
        (1) theta [Vector<float> 1x3]: Required angle of rotation in each axis in radians. 
        (2) axes_sequence_cfg [string]: Rotation axis sequence configuration (e.g. 'XYZ').
                Note ('XYZ'):
                    Matrix multiplication - R_{z}(theta_{2}) @ R_{y}(theta_{1}) @ R_{x}(theta_{0})

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix around {z, y, x}-axis.
    """

    return {
        'XYZ': lambda x: Get_Rotation_Matrix('Z', x[2]) @ Get_Rotation_Matrix('Y', x[1]) @ Get_Rotation_Matrix('X', x[0]),
        'XZY': lambda x: Get_Rotation_Matrix('Y', x[1]) @ Get_Rotation_Matrix('Z', x[2]) @ Get_Rotation_Matrix('X', x[0]),
        'YXZ': lambda x: Get_Rotation_Matrix('Z', x[2]) @ Get_Rotation_Matrix('X', x[0]) @ Get_Rotation_Matrix('Y', x[1]),
        'YZX': lambda x: Get_Rotation_Matrix('X', x[0]) @ Get_Rotation_Matrix('Z', x[2]) @ Get_Rotation_Matrix('Y', x[1]),
        'ZXY': lambda x: Get_Rotation_Matrix('Y', x[1]) @ Get_Rotation_Matrix('X', x[0]) @ Get_Rotation_Matrix('Z', x[2]),
        'ZYX': lambda x: Get_Rotation_Matrix('X', x[0]) @ Get_Rotation_Matrix('Y', x[1]) @ Get_Rotation_Matrix('Z', x[2]),
    }[axes_sequence_cfg](theta)

class Vector3_Cls(object):
    """
    Description:
        A specific class for working with three-dimensional (3D) vector.

        Representation:
            Vector_{x, y, z} = [v_{x}, v_{y}, v_{z}]

    Initialization of the Class:
        Args:
            (1) v [Vector<float> 1x3]: Vector (x, y, z) or None.
                                        Note:
                                            v = None: 
                                                Automatically generated zero vector 
                                                inside the class.
            (2) data_type [np.{float, int, etc.}]: The desired data-type for the vector (v: x, y, z).
                                                   Note:
                                                    Array types from the Numpy library.
        
        Example:
            Initialization:
                # Assignment of the variables.
                v = [0.0, 0.0, 0.0]
                data_type = np.float64

                # Initialization of the class.
                Cls = Vector3_Cls(v, data_type)

            Features:
                # Properties of the class.
                Cls.x; Cls.y; Cls.z; Cls.Shape

                # Functions of the class.
                Cls.Length()
                    ...
                Cls.Normalize()
    """
    
    # Create a global data type for the class.
    cls_data_type = tp.TypeVar('cls_data_type')

    def __init__(self, v: tp.List[float] = None, data_type: cls_data_type = np.float64) -> None:
        try:
            # << PRIVATE >> #
            self.__data_type = data_type

            # Create an array (vector: v) with the desired data type.
            self.__v = self.__data_type(v) if v is not None else self.__data_type([0.0, 0.0, 0.0])

            """
            Description:
                Determine if the shape of the input object is in the 
                correct dimension.
            """
            assert self.__Is_Valid()

            # Assignment the shape of an array.
            self.__shape = np.shape(self.__v)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] The wrong vector shape.')
            print(f'[ERROR] The shape of the vector must be in the form {(3,)}.' )
            
    def __str__(self) -> str:
        """
        Description:
            The method returns the string representation of the object.
            
        Returns:
            (1) parameter [string]: String representation of the object.
        """

        return f'{self.__v}'

    def __getitem__(self, item: int) -> cls_data_type:
        """
        Description:
            The method used to get the value of the item. 

            Note:
                print(self.__v[0])

        Args:
            (1) item [int]: Item (index - i) of the object.

        Returns:
            (1) parameter [data_type]: Value of the item. 
        """

        return self.__v[item]

    def __setitem__(self, item: int, value: tp.Optional[cls_data_type]) -> None:
        """
        Description:
            The method used to assign a value to an item.

            Note:
                self.__v[0] = 0.0
                self.__v[:] = [0.0, 0.0, 0.0]

        Args:
            (1) item [int, mark{:, -1, etc.}]: Item (index - i) of the object.
            (2) value [data_type or float]: The value to be assigned.
        """

        self.__v[item] = self.__data_type(value)

    def __add__(self, v: tp.List[tp.Optional[cls_data_type]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (+) represents the sum of two objects.

        Args:
            (1) v [Vector<cls_data_type> 1x3]: Input vector.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Output vector (self.v + v).
        """

        if isinstance(v, self.__class__):
            return self.__class__(self.__v + v.all(), self.__data_type)
        else:
            return self.__class__(self.__v + v, self.__data_type)

    def __sub__(self, v: tp.List[tp.Optional[cls_data_type]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (-) represents the difference of two objects.

        Args:
            (1) v [Vector<cls_data_type> 1x3]: Input vector.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Output vector (self.v - v).
        """

        if isinstance(v, self.__class__):
            return self.__class__(self.__v - v.all(), self.__data_type)
        else:
            return self.__class__(self.__v - v, self.__data_type)

    def __mul__(self, v: tp.Tuple[tp.Optional[cls_data_type], tp.List[tp.Optional[cls_data_type]]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (*) represents a vector multiplication of two objects.

        Args:
            (1) v [data_type, Vector<cls_data_type> 1x3]: Input scalar or vector (x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Output vector (self.v * v).     
        """
        
        if isinstance(v, (int, float)):
            return self.__class__(self.__v * v, self.__data_type)
        elif isinstance(v, (list, np.ndarray, self.__class__)):
            return self.__class__([self.x * v[0], self.y * v[1], self.z * v[2]], 
                                  self.__data_type)

    def __rmul__(self, v: tp.Tuple[tp.Optional[cls_data_type], tp.List[tp.Optional[cls_data_type]]]) -> tp.List[cls_data_type]:
        """
        Description:
            The method implements the reverse multiplication operation, i.e. multiplication 
            with swapped operands.

        Args:
            (1) v [data_type, Vector<cls_data_type> 1x3]: Input scalar or vector (x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Output vector (v * self.v).     
        """

        return self.__mul__(v)

    def __neg__(self):
        """
        Description:
            The method to calculate the negation of an object.

            Note:
                The negation operator {-v} on the custom object {v}.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Output vector ((-1) * self.v). 
        """

        return self.__class__(-self.__v, self.__data_type)

    def __eq__(self, v: tp.List[cls_data_type]) -> bool:
        """
        Description:
            The operator (==) compares the value or equality of two objects.

        Args:
            (1) v [Vector<cls_data_type> 1x3]: Input vector.

        Returns:
            (1) parameter [Bool]: The result is 'True' if the objects are 
                                  equal, and 'False' if they are not.
        """

        for _, (in_v_i, cls_v_i) in enumerate(zip(v, self.__v)):
            if in_v_i != cls_v_i:
                return False
        return True

    def __Is_Valid(self) -> bool:
        """
        Description:
            Determine if the shape of the input object is in the correct dimension.

        Return:
            (1) parameter [Bool]: The result is 'True' if the dimensions are 
                                  correct, and 'False' if they are not.
        """

        return np.shape(self.__v) == (3,)

    def all(self) -> tp.List[cls_data_type]:
        """
        Description:
            Getting all values of the class object (vector) in the 
            sequence of axes x, y, z.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Vector (x, y, z) with the desired data 
                                                       type.
        """
        
        return self.__v

    @property
    def Type(self) -> cls_data_type:
        """
        Description:
            Get the data-type of the object elements.

        Returns:
            (1) parameter [data_type]: Data-type.
        """

        return self.__data_type

    @property
    def Shape(self) -> tp.Tuple[int, None]:
        """
        Description:
            Get the shape of the class object (vector).

        Returns:
            (1) parameter [Tuple<int> 1x2]: The lengths of the corresponding array dimensions.     
        """

        return self.__shape

    @property
    def x(self) -> cls_data_type:
        """
        Description:
            Get the x part from the class object (vector).

        Returns:
            (1) parameter [data_type]: Scalar part of vector (x - index: 0) with the desired data 
                                       type.
        """

        return self.__v[0]

    @property
    def y(self) -> cls_data_type: 
        """
        Description:
            Get the y part from the class object (vector).

        Returns:
            (1) parameter [data_type]: Scalar part of vector (y - index: 1) with the desired data 
                                       type.
        """

        return self.__v[1]

    @property
    def z(self) -> cls_data_type: 
        """
        Description:
            Get the z part from the class object (vector).

        Returns:
            (1) parameter [data_type]: Scalar part of vector (z - index: 2) with the desired data 
                                       type.
        """

        return self.__v[2]

    def Norm(self) -> cls_data_type:
        """
        Description:
            Get the norm (length or magnitude) of the vector.

            Equation: 
                ||x||_{2} = sqrt(x_{1}^2 + ... + x_{n}^2)

        Returns:
            (1) parameter [data_type]: The norm (length) of the vector (x, y, z).
        """

        return Mathematics.Euclidean_Norm(self.__v)

    def Normalize(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the normalized (unit) vector.

            Equation:
                v_hat = v / ||v||, 

                where v_hat is a normalized vector, v is a non-zero vector and ||v|| is the 
                norm (length) of v.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Normalized (unit) vector (x, y, z) with the desired 
                                                       data type.
        """

        if self.Norm() < Mathematics.CONST_EPS_64:
            return self
        else:
            return self.__class__(self.__v / self.Norm(), self.__data_type)

    def Cross(self, v: tp.List[tp.Optional[cls_data_type]]) -> tp.List[cls_data_type]:
        """
        Description:
            Get the cross product of two vectors.

            Equation:
                The three scalar components of the resulting vector s = s_{1}i + s_{2}j + s_{3}k = a x b 
                are as follows:

                s_{1} = a_{2}*b_{3} - a_{3}*b_{2}
                s_{2} = a_{3}*b_{1} - a_{1}*b_{3}
                s_{3} = a_{1}*b_{2} - a_{2}*b_{1}

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Vector (self.__v x v) with the desired 
                                                       data type.
        """

        return self.__class__(Mathematics.Cross(self.__v, v), self.__data_type)

    def Dot(self, v: tp.List[tp.Optional[cls_data_type]]) -> cls_data_type:
        """
        Description:
            Get the dot product of two vectors.

            Equation:
                self.__v * v = sum(self.__v_{i} * v_{i}), i .. n,

                where self.__v and v are vectors, n is the dimension of the vector 
                space.

        Returns:
            (1) parameter [data_type]: The dot product of two vectors.
        """

        return self.x * v[0] + self.y * v[1] + self.z * v[2]

def Get_Euler_XYZ_From_Matrix(T: tp.List[tp.List[float]]) -> tp.List[float]:
    """
    Description:
        Get the Euler angles from the homogeneous transformation matrix {M} for the 'XYZ' axis sequence.

        Note 1:
            M = [ cos(th_{1}) * cos(th_{2}), sin(th_{0}) * sin(th_{1}) * cos(th_{2}) - sin(th_{2}) * cos(th_{0}),        sin(th_{0}) * sin(th_{2}) + sin(th_{1}) * cos(th_{0}) * cos(th_{2}), 0.0]
                [ sin(th_{2}) * cos(th_{1}), sin(th_{0}) * sin(th_{1}) * sin(th_{2}) + cos(th_{0}) * cos(th_{2}), (-1) * sin(th_{0}) * cos(th_{2}) + sin(th_{1}) * sin(th_{2}) * cos(th_{0}), 0.0]
                [        (-1) * sin(th_{1}),                                           sin(th_{0}) * cos(th_{1}),                                                  cos(th_{0}) * cos(th_{1}), 0.0]
                [                       0.0,                                                                 0.0,                                                                        0.0, 1.0]
            
            where th_{i} represents an angle with individual index (0: x, 1: y, 2: z).

        Note 2:
            cos(0.0) = 1.0; sin(0.0) = 0.0
            cos(1.57) ~ 0.0; cos(-1.57) ~ 0.0
            sin(1.57) = 0.99; sin(-1.57) = -0.99
    Args:
        (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix.

    Returns:
        (1) parameter [Vector<float> 1x3]: Euler angles in the sequence {x, y, z}.
    """

    # Initialization of the Euler angle theta for the ZYX axis sequence.
    theta = Euler_Angle_Cls([0.0] * 3, 'XYZ', np.float64)

    # Numerical instability: 
    #   Note: Paul S. Heckbert (Ed.). 1994. Graphics gems IV. Academic Press Professional, Inc., USA.
    #   Note: cos((-1) * 1.5708) or cos((+1) * 1.5708) is equal to -3.673205103346574e-06.
    # Pythagorean theorem: c^2 = a^2 + b^2
    c_th_1 = np.sqrt(T[2, 1]**2 + T[2, 2]**2)
    
    # Check if the cos(th_{1}) is close to zero (machine epsilon - more information about this can be 
    #                                            found at the top in the constants).
    #   Note: The function atan2(y, x) returns the four-quadrant inverse tangent tan^(-1)(y/x) of y and x.
    #       x = r * cos(th)
    #       y = r * sin(th)
    if c_th_1 > Mathematics.CONST_EPS_64:
        # Note:
        #   sin(th_{0}) * cos(th_{1}) / cos(th_{0}) * cos(th_{1}) -> sin(th_{0}) / cos(th_{0})
        theta[0] = np.arctan2(T[2, 1], T[2, 2])
        # Note:
        #   T[2, 0] = (-1) * sin(th_{1})
        theta[1] = np.arctan2((-1) * T[2, 0], c_th_1)
        # Note:
        #    sin(th_{2}) * cos(th_{1}) / cos(th_{1}) * cos(th_{2}) -> sin(th_{2}) / cos(th_{2})
        theta[2] = np.arctan2(T[1, 0], T[0, 0])
    else:
        # Note:
        #   (-1) * sin(th_{0}) / cos(th_{0})
        theta[0] = np.arctan2((-1) * T[1, 2], T[1, 1])
        # Note:
        #   T[0, 2] = (-1) * sin(th_{1})
        theta[1] = np.arctan2((-1) * T[2, 0], c_th_1)
        theta[2] = 0.0

    return theta

def Get_Euler_ZYX_From_Matrix(T: tp.List[tp.List[float]]) -> tp.List[float]:
    """
    Description:
        Get the Euler angles from the homogeneous transformation matrix {M} for the 'ZYX' axis sequence.

        Note 1:
            M = [                                           cos(th_{1}) * cos(th_{2}),                                           (-1) * sin(th_{2}) * cos(th_{1}),                      sin(th_{1}), 0.0]
                [ sin(th_{0}) * sin(th_{1}) * cos(th_{2}) + sin(th_{2}) * cos(th_{0}), (-1) * sin(th_{0}) * sin(th_{1}) * sin(th_{2}) + cos(th_{0}) * cos(th_{2}), (-1) * sin(th_{0}) * cos(th_{1}), 0.0]
                [ sin(th_{0}) * sin(th_{2}) - sin(th_{1}) * cos(th_{0}) * cos(th_{2}),        sin(th_{0}) * cos(th_{2}) + sin(th_{1}) * sin(th_{2}) * cos(th_{0}),        cos(th_{0}) * cos(th_{1}), 0.0]
                [                                                                 0.0,                                                                        0.0,                              0.0, 1.0]
            
            where th_{i} represents an angle with individual index (0: x, 1: y, 2: z).

        Note 2:
            cos(0.0) = 1.0; sin(0.0) = 0.0
            cos(1.57) ~ 0.0; cos(-1.57) ~ 0.0
            sin(1.57) = 0.99; sin(-1.57) = -0.99
    Args:
        (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix.

    Returns:
        (1) parameter [Vector<float> 1x3]: Euler angles in the sequence {x, y, z}.
    """

    # Initialization of the Euler angle theta for the ZYX axis sequence.
    theta = Euler_Angle_Cls([0.0] * 3, 'ZYX', np.float64)

    # Numerical instability: 
    #   Note: Paul S. Heckbert (Ed.). 1994. Graphics gems IV. Academic Press Professional, Inc., USA.
    #   Note: cos((-1) * 1.5708) or cos((+1) * 1.5708) is equal to -3.673205103346574e-06.
    # Pythagorean theorem: c^2 = a^2 + b^2
    c_th_1 = np.sqrt(T[0, 0]**2 + T[0, 1]**2)
    
    # Check if the cos(th_{1}) is close to zero (machine epsilon - more information about this can be 
    #                                            found at the top in the constants).
    #   Note: The function atan2(y, x) returns the four-quadrant inverse tangent tan^(-1)(y/x) of y and x.
    #       x = r * cos(th)
    #       y = r * sin(th)
    if c_th_1 > Mathematics.CONST_EPS_64:
        # Note:
        #   (-1) * sin(th_{2}) * cos(th_{1}) / cos(th_{1}) * cos(th_{2}) -> (-1) * sin(th_{2}) / cos(th_{2})
        theta[2] = np.arctan2((-1) * T[0, 1], T[0, 0])
        # Note:
        #   T[0, 2] = sin(th_{1}) 
        theta[1] = np.arctan2(T[0, 2], c_th_1)
        # Note:
        #   (-1) * sin(th_{0}) * cos(th_{1}) / cos(th_{0}) * cos(th_{1}) ->  (-1) * sin(th_{0}) / cos(th_{0})
        theta[0] = np.arctan2((-1) * T[1, 2], T[2, 2])
    else:
        # Note:
        #   sin(th_{2}) / cos(th_{2})
        theta[2] = np.arctan2(T[1, 0], T[1, 1])
        # Note:
        #   T[0, 2] = sin(th_{1})
        theta[1] = np.arctan2(T[0, 2], c_th_1)
        theta[0] = 0.0

    return theta

def Get_Angle_Axis_From_Matrix(T: tp.List[tp.List[float]]) -> tp.Tuple[tp.List[float], float]:
    """
    Description:
        Get the Angle-Axis representation of the orientation from the homogeneous transformation matrix {T}.

        Angle:
            alpha = cos^(-1)((r_{0, 0} +  r_{1, 1} + r_{3, 3} - 1.0)/2.0)
        Axis: 
            v_hat = (1.0 / 2.0 * sin(alpha)) * [[r_{2, 1} - r{1, 2}]
                                                [r_{0, 2} - r{2, 0}],
                                                [r_{1, 0} - r{0, 1}]]
    Args:
        (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix.
    
    Returns:
        (1) parameter [float]: Angle of rotation.
        (2) parameter [Vector<float> 1x3]: Direction of the axis of rotation.
    """

    alpha = np.arccos((T[0, 0] + T[1, 1] + T[2, 2] - 1.0)/2.0)
    if alpha == 0:
        # If alpha = 0, then any unit-length direction vector for the axis is valid 
        # because there is no rotation.
        return (alpha, [0.0, 0.0, 0.0])
    else:
        return (alpha, (1.0/(2.0 * np.sin(alpha)) * Vector3_Cls([T[2, 1] - T[1, 2], 
                                                                 T[0, 2] - T[2, 0], 
                                                                 T[1, 0] - T[0, 1]], np.float64)).Normalize())

def Get_Matrix_From_Angle_Axis(alpha: float, v: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the homogeneous transformation matrix {T} from the Angle-Axis representation of orientation.

        The elements of the matrix R(alpha, v_hat) describe the rotation by the single angle {alpha} around fixed axis that lies 
        along the unit vector {v_hat}.

        Unit Vector:
            v_hat = [v_{x}, v_{y}, v{z}]^T

        General representation:
            R(alpha, v_hat) = cos(alpha) * I + sin(alpha) * [k]_{x} + (1 - cos(alpha)) * (k \otimes k),

            where {[k]_{x}} is the cross-product skew symmetric matrix associated with {v_hat = [v_{x}, v_{y}, v_{z}]^T}, (v \otimes v) is the outer 
            product of two coordinate vectors.

        Simplification of the general representation:
            R(alpha, v_hat) = [[                     v_hat_{x}**2 * v_th + c_th, v_hat_{x} * v_hat_{y} * v_th - v_hat_{z} * s_th, v_hat_{x} * v_hat_{z} * v_th + v_hat_{y} * s_th, 0.0],
                               [v_hat_{x} * v_hat_{y} * v_th + v_hat_{z} * s_th,                      v_hat_{y}**2 * v_th + c_th, v_hat_{y} * v_hat_{z} * v_th - v_hat_{x} * s_th, 0.0],
                               [v_hat_{x} * v_hat_{z} * v_th - v_hat_{y} * s_th, v_hat_{y} * v_hat_{z} * v_th + v_hat_{x} * s_th,                    k  _hat_{z}**2 * v_th + c_th, 0.0],
                               [                                            0.0,                                             0.0,                                             0.0, 1.0]],

            where v_th is equal to 1 - cos(alpha) and c, s are equal to cosine, sine functions.

    Args:
        (1) alpha [float]: Angle of rotation.
        (2) v [Vector<float> 1x3]: Direction of the axis of rotation.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix.
    """

    # Note:
    #   R(alpha, v_hat) is explicitly in the form of a 3x3 matrix, but in our case 
    #   we use the homogeneous transformation of a 4x4 matrix.
    if np.abs(alpha) < Mathematics.CONST_EPS_64:
        return Get_Matrix_Identity(4)
    else:
        c_th = np.cos(alpha); s_th = np.sin(alpha)
        v_th = 1 - c_th
        v_hat = Vector3_Cls(v, np.float64).Normalize()

        return Homogeneous_Transformation_Matrix_Cls([[                   v_hat[0]**2 * v_th + c_th, v_hat[0] * v_hat[1] * v_th - v_hat[2] * s_th, v_hat[0] * v_hat[2] * v_th + v_hat[1] * s_th, 0.0],
                                                      [v_hat[0] * v_hat[1] * v_th + v_hat[2] * s_th,                    v_hat[1]**2 * v_th + c_th, v_hat[1] * v_hat[2] * v_th - v_hat[0] * s_th, 0.0],
                                                      [v_hat[0] * v_hat[2] * v_th - v_hat[1] * s_th, v_hat[1] * v_hat[2] * v_th + v_hat[0] * s_th,                    v_hat[2]**2 * v_th + c_th, 0.0],
                                                      [                                         0.0,                                          0.0,                                          0.0, 1.0]], np.float64)

def Get_Quaternion_From_Matrix(T: tp.List[tp.List[float]]) -> tp.List[float]:
    """
    Description:
        Get the Quaternion (w, x, y, z) representation of the orientation from the homogeneous transformation 
        matrix {T} using Shepperd's method.

        Reference:
            S.W. Shepperd, 'Quaternion from rotation matrix,' Journal of Guidance and
            Control, Vol. 1, No. 3, pp. 223-224, 1978.
    
    Args:
        (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix.

    Returns:
        (1) parameter [Vector<float> 1x4]: Output vector as a quaternion (w, x, y, z) with the desired 
                                           data type.
    """

    if isinstance(T, (list, np.ndarray)):
        T = Homogeneous_Transformation_Matrix_Cls(T, np.float64)

    # Get the rotation part from the homogeneous transformation matrix {T}.
    R = T.R

    # Get the ordinal number of the largest element in the vector.
    #   Note:
    #       The i-th solution is considered to be the best solution.
    i = Mathematics.Max(np.array([T.Trace() - 1.0, R[0, 0], R[1, 1], R[2, 2]], 
                                  dtype=np.float64))[0]
    
    # Initilization of the output quaternion (q = {w, x, y, z}).
    q = Quaternion_Cls(None, T.Type)

    if i == 0:
        temp_part = (1 + R[0, 0] + R[1, 1] + R[2, 2]) ** 0.5
        q[0] = temp_part
        q[1] = (R[2, 1] - R[1, 2]) / temp_part
        q[2] = (R[0, 2] - R[2, 0]) / temp_part
        q[3] = (R[1, 0] - R[0, 1]) / temp_part
        return (q * 0.5).Normalize()

    if i == 1:
        temp_part = (1 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5
        q[0] = (R[2, 1] - R[1, 2]) / temp_part
        q[1] = temp_part
        q[2] = (R[0, 1] + R[1, 0]) / temp_part
        q[3] = (R[2, 0] + R[0, 2]) / temp_part
        return (q * 0.5).Normalize()

    if i == 2:
        temp_part = (1 - R[0, 0] + R[1, 1] - R[2, 2]) ** 0.5
        q[0] = (R[0, 2] - R[2, 0]) / temp_part
        q[1] = (R[0, 1] + R[1, 0]) / temp_part
        q[2] = temp_part
        q[3] = (R[1, 2] + R[2, 1]) / temp_part
        return (q * 0.5).Normalize()

    if i == 3:
        temp_part = (1 - R[0, 0] - R[1, 1] + R[2, 2]) ** 0.5
        q[0] = (R[1, 0] - R[0, 1]) / temp_part
        q[1] = (R[2, 0] + R[0, 2]) / temp_part
        q[2] = (R[2, 1] + R[1, 2]) / temp_part
        q[3] = temp_part
        return (q * 0.5).Normalize()

def Get_Quaternion_From_Angle_Axis(alpha: float, v: tp.List[float]) -> tp.List[float]:
    """
    Description:
        Get the Quaternion (w, x, y, z) from the Angle-Axis representation of orientation.

        General representation:
            q(alpha, v) = [cos(alpha / 2.0),
                           v_hat * sin(alpha / 2.0)]

    Args:
        (1) alpha [float]: Angle of rotation.
        (2) v [Vector<float> 1x3]: Direction of the axis of rotation.

    Returns:
        (1) parameter [Vector<float> 1x4]: Output vector as a quaternion (w, x, y, z) with the desired 
                                           data type.
    """
    
    if isinstance(v, (list, np.ndarray)):
        v = Vector3_Cls(v, np.float64)

    alpha_half = alpha * 0.5
    return Quaternion_Cls(np.append(np.cos(alpha_half), v.Normalize().all() * np.sin(alpha_half)).tolist(), v.Type).Normalize()

class Homogeneous_Transformation_Matrix_Cls(object):
    """
    Description:
        A specific class for working with homogeneous transformation matrix {T}.

        Representation:
            T = [[R_{0, 0}, R_{0, 1}, R_{0, 2}, p_{0}],
                 [R_{0, 0}, R_{0, 1}, R_{0, 2}, p_{1}],
                 [R_{0, 0}, R_{0, 1}, R_{0, 2}, p_{2}],
                 [     0.0,      0.0,      0.0,   1.0]],

            where R (3x3) is the rotation matrix representing orientation and p (3x1) is the position 
            vector of the matrix representing translation.

    Initialization of the Class:
        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix {T}.
            (2) data_type [np.{float (optional), int, etc.}]: The desired data-type for the homogeneous 
                                                              transformation matrix {T}. 
                                                   
                                                              Note:
                                                                Array types from the Numpy library.
            
        Example:
            Initialization:
                # Assignment of the variables.
                T = [[1.0, 0.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0, 0.0],
                     [0.0, 0.0, 1.0, 0.0],
                     [0.0, 0.0, 0.0, 1.0]]
                data_type = np.float64
                
                # Initialization of the class.
                Cls = Homogeneous_Transformation_Matrix_Cls(T, data_type)

            Features:
                # Properties of the class.
                Cls.p; Cls.R; Cls.Shape

                # Functions of the class.
                Cls.Transpose()
                Cls.Diagonal()
                    ...
                Cls.Get_Rotation('ZYX')
    """
    
    # Create a global data type for the class.
    cls_data_type = tp.TypeVar('cls_data_type')

    def __init__(self, T: tp.List[tp.List[float]] = None, data_type: cls_data_type = np.float64) -> None:
        try: 
            # << PRIVATE >> #
            self.__data_type = data_type

            # Create an array with the desired data type.
            self.__T = self.__data_type(T) if T is not None else self.__data_type(Get_Matrix_Identity(4))

            """
            Description:
                Determine if the shape of the input object is in the 
                correct dimension.
            """
            assert self.__Is_Valid()

            # Assignment the shape of an array.
            self.__shape = np.shape(self.__T)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] The wrong matrix shape.')
            print(f'[ERROR] The shape of the matrix must be in the form {(4, 4)}.' )

    def __str__(self) -> str:
        """
        Description:
            The method returns the string representation of the object.
            
        Returns:
            (1) parameter [string]: String representation of the object.
        """

        return f'{self.__T}'

    def __getitem__(self, item: tp.Union[int, tp.Tuple[int, int]]) -> tp.Union[cls_data_type, tp.List[cls_data_type]]:
        """
        Description:
            The method used to get the value of the item. 

            Note:
                print(self.__T[0])
                print(slef.__T[0, 0])

        Args:
            (1) item [int, (int, int)]: Item (index - i, j) of the object.

        Returns:
            (1) parameter [cls_data_type, Vector<cls_data_type>]: Value of the item. 
        """

        return self.__T[item]

    def __setitem__(self, item: tp.Union[int, tp.Tuple[int, int]], value: tp.Union[cls_data_type, tp.List[cls_data_type]]) -> None:
        """
        Description:
            The method used to assign a value to an item.

            Note:
                self.__T[0]    = [0.0, 0.0, 0.0, 0.0]
                slef.__T[0, 0] = 0.0

        Args:
            (1) item [int, (int, int)]: Item (index - i, j) of the object.
            (2) value [cls_data_type, Vector<cls_data_type> 1x4]: The value or vector of values 
                                                          to be assigned.
        """

        self.__T[item] = self.__data_type(value)

    def __add__(self, T: tp.List[tp.List[tp.Optional[cls_data_type]]]) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            The operator (+) represents the sum of two objects.

        Args:
            (1) T [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix for addition.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix (T + self.T).
        """

        if isinstance(T, self.__class__):
            return self.__class__(self.__T + T.all(), self.__data_type)
        else:
            return self.__class__(self.__T + T, self.__data_type)

    def __sub__(self, T: tp.List[tp.List[tp.Optional[cls_data_type]]]) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            The operator (-) represents the difference of two objects.

        Args:
            (1) T [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix for subtraction.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix (T - self.T).
        """

        if isinstance(T, self.__class__):
            return self.__class__(self.__T - T.all(), self.__data_type)
        else:
            return self.__class__(self.__T - T, self.__data_type)

    def __matmul__(self, T: tp.List[tp.List[tp.Optional[cls_data_type]]]) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            The operator (@ or x) represents a matrix multiplication of two objects.

        Args:
            (1) T [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix for multiplication.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix (T x self.T).

        """

        if isinstance(T, self.__class__):
            return self.__class__(self.__T @ T.all(), self.__data_type)
        else:
            return self.__class__(self.__T @ T, self.__data_type)

    def __eq__(self, T: tp.List[tp.List[cls_data_type]]) -> bool:
        """
        Description:
            The operator (==) compares the value or equality of two objects.

        Args:
            (1) T [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix for comparison.

        Returns:
            (1) parameter [Bool]: The result is 'True' if the objects are 
                                  equal, and 'False' if they are not.
        """

        for _, (in_T_i, cls_T_i) in enumerate(zip(T, self.__T)):
            for _, (in_T_ij, cls_T_ij) in enumerate(zip(in_T_i, cls_T_i)):
                if in_T_ij != cls_T_ij:
                    return False
        return True

    def __Is_Valid(self) -> bool:
        """
        Description:
            Determine if the shape of the input object is in the correct dimension.

        Return:
            (1) parameter [Bool]: The result is 'True' if the dimensions are 
                                  correct, and 'False' if they are not.
        """

        return np.shape(self.__T) == (4, 4)

    def all(self) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get all values of the homogeneous transformation matrix {T}.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix {T} of the class.
        """
        
        return self.__T

    @property
    def Type(self) -> cls_data_type:
        """
        Description:
            Get the data-type of the object elements.

        Returns:
            (1) parameter [data_type]: Data-type.
        """

        return self.__data_type
        
    @property
    def Shape(self) -> tp.Tuple[int, int]:
        """
        Description:
            Get the shape of the input homogeneous transformation matrix {T}.

        Returns:
            (1) parameter [Tuple<int> 1x2]: The lengths of the corresponding array dimensions.     
        """

        return self.__shape

    @property 
    def p(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the translation part from the homogeneous transformation matrix {T}.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Translation part of the matrix.
        """

        return Vector3_Cls(self.__T[:3, 3], self.__data_type)

    @property
    def R(self) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get the rotation part from the homogeneous transformation matrix {T}.

        Returns:
            (1) parameter [Matrix<cls_data_type> 3x3]: Rotation part of the matrix.
        """

        return self.__T[:3, :3]

    def Translation(self, vector: tp.List[tp.Optional[cls_data_type]]) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Translate the homogeneous transformation matrix {T} according to the input 
            direction vector.

        Args:
            (1) vector [Vector<cls_data_type> 1x3]: Direction vector (x, y, z).

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Translated homogeneous transformation matrix.
        """

        self.__T[:3, 3] += vector
        return self.__class__(self.__T, self.__data_type)

    def Rotation(self, angle: tp.List[tp.Optional[cls_data_type]], axes_sequence_cfg: str) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Rotation of the homogeneous transformation matrix from a specified sequence of axes.

            Available axis sequence configurations:
                (1, 2) Euler Angle (Vector<cls_data_type> 1x3): XYZ, ZYX
                (3) Quaternion (Vector<cls_data_type> 1x4): W, X, Y, Z
                (4) Angle-Axis (Vector<cls_data_type> 1x4): Angle + X, Y, Z

            Equation:
                T_{new} = T_{current} @ T_{rotated}(Euler Angle, Quaternion, Angle-Axis)

        Args:
            (1) angle [Vector<cls_data_type> 1x3, 1x4]: Angle of rotation defined in the specified form (axes sequence 
                                                    configuration): Euler Angles, Quaternions, etc.
            (2) axes_sequence_cfg [string]: Rotation axis sequence configuration (e.g. 'ZYX', 'QUATERNION', etc.)

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix rotated around the specified 
                                                   configuration of the axis sequence.
        """

        return {
            'XYZ': lambda x: self.__class__(self.__T @ Euler_Angle_Cls(x, axes_sequence_cfg, self.__data_type).Get_Homogeneous_Transformation_Matrix().all(), self.__data_type),
            'ZYX': lambda x: self.__class__(self.__T @ Euler_Angle_Cls(x, axes_sequence_cfg, self.__data_type).Get_Homogeneous_Transformation_Matrix().all(), self.__data_type),
            'QUATERNION': lambda x: self.__class__(self.__T @ Quaternion_Cls(x, self.__data_type).Get_Homogeneous_Transformation_Matrix('Homogeneous').all(), self.__data_type),
            'ANGLE_AXIS': lambda x: self.__class__(self.__T @ Get_Matrix_From_Angle_Axis(x[0], x[1:]).all(), self.__data_type),
        }[axes_sequence_cfg](angle)

    def Transpose(self) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get the homogeneous transformation matrix {T} with transposed axes.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Transposed matrix.
        """

        T_out = np.zeros((self.__shape[1], self.__shape[0]), dtype=self.__data_type)
        for i, T_out_i in enumerate(self.__T):
            for j, T_out_ij in enumerate(T_out_i):
                T_out[j][i] = T_out_ij
        
        return self.__class__(T_out, self.__data_type)

    def Inverse(self) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get the inverse of the homogeneous transformation matrix {T}.

            The standard form of the homogeneous transformation matrix:
                T = [R_{3x3}, p_{3x1}
                     0_{1x3}, 1_{1x1}],

                where R is a rotation matrix and p is a translation vector.

            The inverse form of the homogeneous transformation matrix:
                T^(-1) = [R^(-1)_{3x3}, -R^(-1)_{3x3} x p_{3x1}
                               0_{1x3},                 1_{1x1}]
            
            Note:
                The inverse of the rotation matrix is equal to its transpose.
                    R^(-1) = R^T
        
        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Inversion of of the homogeneous transformation matrix {T}.
        """
        
        R_inv = Get_Matrix_Tranpose(self.R)
        p_inv = (-R_inv @ self.p.all()).reshape(3,1)

        return self.__class__(np.vstack((np.hstack((R_inv, p_inv)),
                                         [0.0, 0.0, 0.0, 1.0])), self.__data_type)

    def Diagonal(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the vector of elements of the diagonal of the homogeneous transformation 
            matrix {T}.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Vector of elements of the diagonal of the input matrix.
        """

        return Get_Matrix_Diagonal(self.__T)

    def Trace(self) -> cls_data_type:
        """
        Description:
            Get the sum of the elements on the main diagonal (upper left to lower 
            right) of the homogeneous transformation matrix {T}.

            Equation:
                tr(M) = sum(M_{ii})

        Returns:
            (1) parameter [data_type]: Trace of an 4x4 square matrix {T}.
        """

        return Get_Matrix_Trace(self.__T)

    def Scale(self, scale_vector: tp.List[tp.Optional[cls_data_type]]) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Set the scale of the homogeneous transformation matrix according to scale 
            factor of each axis.

        Args:
            (1) scale_vector [Vector<cls_data_type> 1x3]: Scale factor of x, y, z axis.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Scaled homogeneous transformation matrix.
        """

        T_Scaled = self.__T.copy()
        for i in range(self.__shape[0] - 1):
            T_Scaled[i, i] = scale_vector[i]

        return self.__class__(T_Scaled, self.__data_type)

    def Get_Scale_Vector(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the scaling vector from the homogeneous transformation matrix {T}.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Scaling part of the matrix.
        """

        scale_vector = np.zeros(self.__shape[0] - 1, dtype = self.__data_type)    
        for i, R_i in enumerate(self.R):
            scale_vector[i] = Mathematics.Euclidean_Norm(R_i)

        return scale_vector

    def Get_Rotation(self, axes_sequence_cfg: str) -> tp.List[float]:
        """
        Description:
            Get the rotation of the homogeneous transformation matrix in a specified sequence of axes.

            Available axis sequence configurations:
                (1, 2) Euler Angle (Vector<cls_data_type> 1x3): XYZ, ZYX
                (3) Quaternion (Vector<cls_data_type> 1x4): W, X, Y, Z
                (4) Angle-Axis (Vector<cls_data_type> 1x4): Angle + X, Y, Z

        Args:s
            (1) axes_sequence_cfg [string]: Rotation axis sequence configuration (e.g. 'ZYX', 'QUATERNION', etc.)

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3, 1x4]: The required angle of rotation defined in the specified form (axes sequence 
                                                        configuration): Euler Angles, Quaternions, etc.
        """
        
        return {
            'XYZ': lambda x: Get_Euler_XYZ_From_Matrix(x), 
            'ZYX': lambda x: Get_Euler_ZYX_From_Matrix(x),
            'QUATERNION': lambda x: Get_Quaternion_From_Matrix(x),
            'ANGLE_AXIS': lambda x: Get_Angle_Axis_From_Matrix(x)
        }[axes_sequence_cfg](self.__T)

class Euler_Angle_Cls(Vector3_Cls):
    """
    Description:
        A specific class for working with Euler angles represented by rotations 
        about the axes of the coordinate system (x, y, z).

        Note:
            Euler angles are also known as Tait-Bryan / Cardan angles or yaw, pitch, and 
            roll angles.

        Representation:
            theta_{x, y, z} = [theta_{x}, theta_{y}, theta_{z}]

    Initialization of the Class:
        Args:
            (1) theta [Vector<float> 1x3]: Angle of rotation (Euler_{x, y, z}) in radians.
            (2) axes_sequence_cfg [string]: Rotation axis sequence configuration (e.g. 'ZYX').
            (3) data_type [np.{float, int, etc.}]: The desired data-type for the Euler Angle (theta: x, y, z).
                                                   
                                                   Note:
                                                    Array types from the Numpy library.
            
        Example:
            Initialization:
                # Assignment of the variables.
                theta = [0.0, 0.0, 0.0]
                axes_sequence_cfg = 'ZYX'
                data_type = np.float64

                # Initialization of the class.
                Cls = Euler_Angle_Cls(theta, axes_sequence_cfg, data_type)

            Features:
                # Properties of the class (Vector3_Cls).
                Cls.x; Cls.y; Cls.z; Cls.Shape
                Cls.Degree

                # Functions of the class.
                Cls.Set_Axes_Sequence_Cfg('XYZ')
                    ...
                Cls.Get_Homogeneous_Transformation_Matrix('ZYX')
    """
    
    # Create a global data type for the class.
    cls_data_type = tp.TypeVar('cls_data_type')

    def __init__(self, theta: tp.List[float] = None, axes_sequence_cfg: str = 'ZYX', 
                 data_type: cls_data_type = np.float64) -> None:
                 
        # Returns the objects represented in the parent class (Vector3_Cls). 
        super().__init__(theta, data_type)

        # << PRIVATE >> #
        self.__axes_sequence_cfg = axes_sequence_cfg

        # Create an array with the desired data type.
        self.__data_type = self.Type
        self.__theta = self.all()

    @property
    def Degree(self) -> tp.List[cls_data_type]:
        """
        Description:
            Conversion of input object from radians to degrees.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Angle of rotation in degrees.
        """

        return Mathematics.Radian_To_Degree(self.__theta)

    def Set_Axes_Sequence_Cfg(self, axes_sequence_cfg: str):
        """
        Description:
            Set the axis sequence configuration of the input object.

        Args:
            (1) axes_sequence_cfg [string]: Rotation axis sequence configuration (e.g. 'ZYX').
        """

        if axes_sequence_cfg != self.__axes_sequence_cfg:
            self.__axes_sequence_cfg = axes_sequence_cfg

    def Get_Axes_Sequence_Cfg(self) -> str:
        """
        Description:
            Get the axis sequence configuration of the input object.

        Returns:
            (1) parameter [string]: Rotation axis sequence configuration (e.g. 'ZYX').
        """

        return self.__axes_sequence_cfg

    def Get_Homogeneous_Transformation_Matrix(self) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get the homogeneous transformation matrix from the Euler angles in a specified sequence of axes. The fastest way to calculate the desired 
            matrix is obtained by simplifying the standard function 'Get_Matrix_From_Euler_Method_Standard(theta, axes_sequence_cfg)'.

            Note 1:
                For more information on simplifying the function, see:
                    ../Lib/Simplification/Euler_Angles.py

            Note 2:
                axes_sequence_cfg = 'ZYX'

                R_{x}(theta_{0}) @ \
                R_{y}(theta_{1}) @ \
                R_{z}(theta_{2}) = [                                           cos(th_{1}) * cos(th_{2}),                                           (-1) * sin(th_{2}) * cos(th_{1}),                      sin(th_{1}), 0.0]
                                   [ sin(th_{0}) * sin(th_{1}) * cos(th_{2}) + sin(th_{2}) * cos(th_{0}), (-1) * sin(th_{0}) * sin(th_{1}) * sin(th_{2}) + cos(th_{0}) * cos(th_{2}), (-1) * sin(th_{0}) * cos(th_{1}), 0.0]
                                   [ sin(th_{0}) * sin(th_{2}) - sin(th_{1}) * cos(th_{0}) * cos(th_{2}),        sin(th_{0}) * cos(th_{2}) + sin(th_{1}) * sin(th_{2}) * cos(th_{0}),        cos(th_{0}) * cos(th_{1}), 0.0]
                                   [                                                                 0.0,                                                                        0.0,                              0.0, 1.0]

                where th_{i} represents an angle with individual index (0: x, 1: y, 2: z).

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix around {z, y, x}-axis.
        """
        
        # Abbreviations for individual angle functions.
        c_th_x = np.cos(self.__theta[0]); c_th_y = np.cos(self.__theta[1]); c_th_z = np.cos(self.__theta[2])
        s_th_x = np.sin(self.__theta[0]); s_th_y = np.sin(self.__theta[1]); s_th_z = np.sin(self.__theta[2])

        return {
            'XYZ': Homogeneous_Transformation_Matrix_Cls([[ c_th_y * c_th_z, s_th_x * s_th_y * c_th_z - s_th_z * c_th_x,        s_th_x * s_th_z + s_th_y * c_th_x * c_th_z, 0.0],
                                                          [ s_th_z * c_th_y, s_th_x * s_th_y * s_th_z + c_th_x * c_th_z, (-1) * s_th_x * c_th_z + s_th_y * s_th_z * c_th_x, 0.0],
                                                          [   (-1) * s_th_y,                            s_th_x * c_th_y,                                   c_th_x * c_th_y, 0.0],
                                                          [             0.0,                                        0.0,                                               0.0, 1.0]], self.__data_type),
            'ZYX': Homogeneous_Transformation_Matrix_Cls([[                            c_th_y * c_th_z,                            (-1) * s_th_z * c_th_y,                 s_th_y, 0.0],
                                                          [ s_th_x * s_th_y * c_th_z + s_th_z * c_th_x, (-1) * s_th_x * s_th_y * s_th_z + c_th_x * c_th_z, (-1) * s_th_x * c_th_y, 0.0],
                                                          [ s_th_x * s_th_z - s_th_y * c_th_x * c_th_z,        s_th_x * c_th_z + s_th_y * s_th_z * c_th_x,        c_th_x * c_th_y, 0.0],
                                                          [                                        0.0,                                               0.0,                    0.0, 1.0]], self.__data_type)
        }[self.__axes_sequence_cfg]

    def Get_Quaternion(self):
        """
        Description:
            Get the Unit-Quaternion (w, x, y, z) from the Euler angles in a specified sequence of axes.
        
        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (w, x, y, z) with the desired 
                                                       data type.
        """

        # Recalculate the angle of rotation for the conversion (Euler Angles -> Quaternion).
        th_half = self.__theta * 0.5

        # Abbreviations for individual angle functions.
        c_th_x = np.cos(th_half[0]); c_th_y = np.cos(th_half[1]); c_th_z = np.cos(th_half[2])
        s_th_x = np.sin(th_half[0]); s_th_y = np.sin(th_half[1]); s_th_z = np.sin(th_half[2])

        return {
            'XYZ': Quaternion_Cls([c_th_x * c_th_y * c_th_z + s_th_x * s_th_y * s_th_z,
                                   s_th_x * c_th_y * c_th_z - c_th_x * s_th_y * s_th_z,
                                   c_th_x * s_th_y * c_th_z + s_th_x * c_th_y * s_th_z,
                                   c_th_x * c_th_y * s_th_z - s_th_x * s_th_y * c_th_z], self.__data_type).Normalize(),
            'ZYX': Quaternion_Cls([c_th_x * c_th_y * c_th_z - s_th_x * s_th_y * s_th_z,
                                   s_th_x * c_th_y * c_th_z + c_th_x * s_th_y * s_th_z,
                                   c_th_x * s_th_y * c_th_z - s_th_x * c_th_y * s_th_z,
                                   c_th_x * c_th_y * s_th_z + s_th_x * s_th_y * c_th_z], self.__data_type).Normalize()
        }[self.__axes_sequence_cfg]

class Quaternion_Cls(object):
    """
    Description:
        A specific class for working with Quaternions to express three-dimensional (3D) orientation 
        in the sequence of axes (w, x, y, z).  

        In general, the quaternion q can be defined as the sum of the scalar w (q_{0}) and the 
        vector v (q_{1:3}). More precisely below.

        Note:
            Quaternions were first described by William Rowan Hamilton, a 19th century 
            Irish mathematician. 

        A quaternion is an expression of the form:
            q = w + v = w + v_{0}i_hat + v_{1}j_hat + v_{2}k_hat, 
            
            where w is a scalar, v (x, y, z) is a vector and {i, j, k}_hat are unit vectors.

            Note:
                i_hat = (1, 0, 0); j_hat = (0, 1, 0); k_hat = (0, 0, 1).

    Initialization of the Class:
        Args:
            (1) q [Vector<cls_data_type> 1x4]: Input vector as a quaternion in the sequence of axes (w, x, y, z).
            (2) data_type [np.{float, int, etc.}]: The desired data-type for the Quaternion (q: w, x, y, z).
                                                   
                                                   Note:
                                                    Array types from the Numpy library.
            
        Example:
            Initialization:
                # Assignment of the variables.
                q = [1.0, 0.0, 0.0, 0.0]
                data_type = np.float64

                # Initialization of the class.
                Cls = Quaternion_Cls(q, data_type)

            Features:
                # Properties of the class (Quaternion_Cls).
                Cls.w; Cls.x; Cls.y; Cls.z; Cls.Shape
                Cls.Type

                # Functions of the class.
                Cls.Distance('Euclidean', q), Cls.Exponential()
                    ...
                Cls.Get_Homogeneous_Transformation_Matrix()
    """

    # Create a global data type for the class.
    cls_data_type = tp.TypeVar('cls_data_type')

    def __init__(self, q: tp.List[float] = None, data_type: cls_data_type = np.float64) -> None:
        try:
            # << PRIVATE >> #
            self.__data_type = data_type

            # Create an array (quaternion: q) with the desired data type.
            self.__q = self.__data_type(q) if q is not None else self.__data_type([1.0, 0.0, 0.0, 0.0])

            """
            Description:
                Determine if the shape of the input object is in the 
                correct dimension.
            """
            assert self.__Is_Valid()

            # Assignment the shape of an array.
            self.__shape = np.shape(self.__q)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] The wrong quaternion shape.')
            print(f'[ERROR] The shape of the quaternion must be in the form {(4,)}.' )
            
    def __str__(self) -> str:
        """
        Description:
            The method returns the string representation of the object.
            
        Returns:
            (1) parameter [string]: String representation of the object.
        """

        return f'{self.__q}'

    def __getitem__(self, item: int) -> cls_data_type:
        """
        Description:
            The method used to get the value of the item. 

            Note:
                print(self.__q[0])

        Args:
            (1) item [int]: Item (index - i) of the object.

        Returns:
            (1) parameter [data_type]: Value of the item. 
        """

        return self.__q[item]

    def __setitem__(self, item: int, value: tp.Optional[cls_data_type]) -> None:
        """
        Description:
            The method used to assign a value to an item.

            Note:
                self.__q[0] = 0.0
                self.__q[:] = [1.0, 0.0, 0.0, 0.0]

        Args:
            (1) item [int, mark{:, -1, etc.}]: Item (index - i) of the object.
            (2) value [data_type or float]: The value to be assigned.
        """

        self.__q[item] = self.__data_type(value)

    def __add__(self, q: tp.List[tp.Optional[cls_data_type]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (+) represents the sum of two objects.

        Args:
            (1) q [Vector<cls_data_type> 1x4]: Input vector as a quaternion (w, x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (self.q + q).
        """

        if isinstance(q, self.__class__):
            return self.__class__(self.__q + q.all(), self.__data_type)
        else:
            return self.__class__(self.__q + q, self.__data_type)

    def __sub__(self, q: tp.List[tp.Optional[cls_data_type]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (-) represents the difference of two objects.

        Args:
            (1) q [Vector<cls_data_type> 1x4]: Input vector as a quaternion (w, x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (self.q - q).
        """

        if isinstance(q, self.__class__):
            return self.__class__(self.__q - q.all(), self.__data_type)
        else:
            return self.__class__(self.__q - q, self.__data_type)

    def __mul__(self, q: tp.Tuple[tp.Optional[cls_data_type], tp.List[tp.Optional[cls_data_type]]]) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (*) represents a quaternion multiplication of two objects (also called Hamilton product).

            Note:
                The quaternion product is not commutative:
                    self.q * q != q * self.q

        Args:
            (1) q [data_type, Vector<cls_data_type> 1x4]: Input scalar or quaternion (w, x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (self.q * q).    
        """
        
        if isinstance(q, float):
            return self.__class__(self.__q * self.__data_type(q), self.__data_type)
        elif isinstance(q, (list, np.ndarray, self.__class__)):
            return self.__class__([self.w * q[0] - self.x * q[1] - self.y * q[2] - self.z * q[3], 
                                   self.w * q[1] + self.x * q[0] + self.y * q[3] - self.z * q[2], 
                                   self.w * q[2] - self.x * q[3] + self.y * q[0] + self.z * q[1], 
                                   self.w * q[3] + self.x * q[2] - self.y * q[1] + self.z * q[0]], 
                                  self.__data_type)

    def __rmul__(self, q: tp.Tuple[tp.Optional[cls_data_type], tp.List[tp.Optional[cls_data_type]]]) -> tp.List[cls_data_type]:
        """
        Description:
            The method implements the reverse multiplication operation, i.e. multiplication 
            with swapped operands.

        Args:
            (1) q [data_type, Vector<cls_data_type> 1x4]: Input scalar or quaternion (w, x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (q * self.q).    
        """

        return self.__mul__(q)

    def __neg__(self) -> tp.List[cls_data_type]:
        """
        Description:
            The method to calculate the negation of an object.

            Note:
                The negation operator {-q} on the custom object {q}.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion ((-1) * self.q). 
        """

        return self.__class__(-self.__q, self.__data_type)

    def __eq__(self, q: tp.List[cls_data_type]) -> bool:
        """
        Description:
            The operator (==) compares the value or equality of two objects.

        Args:
            (1) q [Vector<cls_data_type> 1x4]: Input vector as a quaternion (w, x, y, z).

        Returns:
            (1) parameter [Bool]: The result is 'True' if the objects are 
                                  equal, and 'False' if they are not.
        """

        for _, (in_v_i, cls_v_i) in enumerate(zip(q, self.__q)):
            if in_v_i != cls_v_i:
                return False
        return True

    def __pow__(self, x: int) -> tp.List[cls_data_type]:
        """
        Description:
            The operator (** or pow) represents the exponential operation for the quaternion (w, x, y, z).

            The exponential of q is defined as:
                q^x = ||q||^x * (cos(x*theta) + q.Vector/||q.Vector|| * sin(x*theta)),

                where theta is equal to arccos(q_{0}/|q|).
        Args:
            (1) x [int]: The scalar (element) as the exponent part of the equation.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (self.q^x) raised 
                                                       by the exponent of x.
        """

        theta = np.arccos(self.Scalar/self.Norm())
        return (self.Norm()**x) * self.__class__(np.append(np.cos(x * theta), self.Vector.Normalize().all() * np.sin(x * theta)).tolist(), 
                                                 self.__data_type)

    def __Is_Valid(self) -> bool:
        """
        Description:
            Determine if the shape of the input object is in the correct dimension.

        Return:
            (1) parameter [Bool]: The result is 'True' if the dimensions are 
                                  correct, and 'False' if they are not.
        """

        return np.shape(self.__q) == (4,)

    def all(self) -> tp.List[cls_data_type]:
        """
        Description:
            Getting all values of the class object (quaternion) in the 
            sequence of axes w, x, y, z.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (w, x, y, z) with the desired 
                                                       data type.
        """
        
        return self.__q

    @property
    def Type(self) -> cls_data_type:
        """
        Description:
            Get the data-type of the object elements.

        Returns:
            (1) parameter [data_type]: Data-type.
        """

        return self.__data_type

    @property
    def Shape(self) -> tp.Tuple[int, None]:
        """
        Description:
            Get the shape of the class object (quaternion).

        Returns:
            (1) parameter [Tuple<int> 1x2]: The lengths of the corresponding array dimensions.     
        """

        return self.__shape

    @property
    def w(self) -> cls_data_type:
        """
        Description:
            Get the w part from the class object (quaternion).

        Returns:
            (1) parameter [data_type]: Scalar part of quaternion (w - index: 0) with the desired data 
                                       type.
        """

        return self.__q[0]

    @property
    def x(self) -> cls_data_type:
        """
        Description:
            Get the x part from the class object (quaternion).

        Returns:
            (1) parameter [data_type]: Scalar part of quaternion (x - index: 1) with the desired data 
                                       type.
        """

        return self.__q[1]

    @property
    def y(self) -> cls_data_type:
        """
        Description:
            Get the y part from the class object (quaternion).

        Returns:
            (1) parameter [data_type]: Scalar part of quaternion (y - index: 2) with the desired data 
                                       type.
        """

        return self.__q[2]

    @property
    def z(self) -> cls_data_type:
        """
        Description:
            Get the z part from the class object (quaternion).

        Returns:
            (1) parameter [data_type]: Scalar part of quaternion (z - index: 3) with the desired data 
                                       type.
        """

        return self.__q[3]

    @property
    def Scalar(self) -> cls_data_type:
        """
        Description:
            Get the scalar (w: real part) from the class object (quaternion).

         Returns:
            (1) parameter [data_type]: Scalar part of the quaternion (w) with the 
                                       desired data type. 
        """

        return self.__q[0]

    @property
    def Vector(self) -> tp.List[cls_data_type]: 
        """
        Description:
            Get the vector (x, y, z: imaginary part) from the class object (quaternion).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3]: Vector part of the quaternion (x, y, z) with the 
                                                       desired data type. 
        """
        
        return Vector3_Cls(self.__q[1:], self.__data_type)

    def Norm(self) -> cls_data_type:
        """
        Description:
            Get the norm (length or magnitude) of the quaternion.

            Equation: 
                ||x||_{2} = sqrt(x_{1}^2 + ... + x_{n}^2)

        Returns:
            (1) parameter [data_type]: The norm (length) of the quaternion (w, x, y, z).
        """

        return Mathematics.Euclidean_Norm(self.__q)

    def Normalize(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the normalized (unit) quaternion.

            Converts a quaternion to a quaternion with the same orientation but with magnitude 1.0.

            Equation:
                q_hat = q / ||q||, 

                where q_hat is a normalized vector, v is a non-zero vector and ||q|| is the 
                norm (length) of q.

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Normalized (unit) vector as a quaternion (w, x, y, z) with the desired 
                                                       data type.
        """

        if self.Norm() < Mathematics.CONST_EPS_64:
            return self
        else:
            return self.__class__(self.__q / self.Norm(), self.__data_type)

    def Conjugate(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the conjugate of the quaternion.

            Equation:
                q* = (q_{0} + q_{1} + q_{2} + q_{3})* = q_{0} - q_{1} - q_{2} - q_{3}

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output (conjugated) vector as a quaternion (w, x, y, z) with the desired 
                                                       data type.
        """

        return self.__class__([self.w, -self.x, -self.y, -self.z], self.__data_type)

    def Inverse(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the inverse of the quaternion.

            Equation:
                q^(-1) = q*/||q||^2

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output (inverse) vector as a quaternion (w, x, y, z) with the desired 
                                                       data type.
        """

        return self.__class__(self.Conjugate().all()/self.Norm()**2, self.__data_type)

    def Dot(self, q: tp.List[cls_data_type]) -> cls_data_type:
        """
        Description:
            Get the dot product of two quaternions.

            Equation:
                self.__q * q = sum(self.__v_{i} * v_{i}), i .. n,

                where self.__q and q are quaternions, n is the dimension of the quaternion 
                space.

        Args:
            (1) q [Vector<cls_data_type> 1x4]]: Input vector as a quaternion (w, x, y, z).

        Returns:
            (1) parameter [cls_data_type]: The dot product of two quaternions.
        """

        return self.__q[0] * q[0] + self.__q[1] * q[1] + self.__q[2] * q[2] + self.__q[3] * q[3]

    def Difference(self, q: tp.List[cls_data_type]) -> tp.List[cls_data_type]:
        """
        Description:
             Get the difference between two quaternions.

        Args:
            (1) q [Vector<cls_data_type> 1x4]: Input vector as a quaternion (w, x, y, z).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output vector as a quaternion (w, x, y, z) with the desired 
                                                       data type. Difference of two quaternions.
        """

        if isinstance(q, (list, np.ndarray)):
            q = self.__class__(q, self.__data_type)

        # Get the normalized (unit) quaternion from the 
        # input quaternion.
        q.Normalize(); self.Normalize()

        return self * q.Inverse()

    def Logarithm(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the logarithm {ln(q)} of the quaternion (w, x, y, z).

            The logarithm of q is defined as:
                ln(q) = ln(||q||) + q.Vector/||q.Vector|| * arccos(q_{0}/||q||)

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output (logarithmic) vector as a quaternion (w, x, y, z) with the desired 
                                                       data type. 
        """

        q_norm = self.Norm()
        if q_norm > Mathematics.CONST_EPS_64:
            return self.__class__(np.append(np.log(q_norm), self.Vector.Normalize().all() * np.arccos(self.__q[0]/q_norm)).tolist(), 
                                            self.__data_type)
        else:
            return self.__class__([np.log(q_norm), 0.0, 0.0, 0.0], self.__data_type)

    def Exponential(self) -> tp.List[cls_data_type]:
        """
        Description:
            Get the exponential {exp(q)} of the quaternion (w, x, y, z).

            The exponential of q is defined as:
                e^q = e^q_{0} * (cos(||q.Vector||) + q.Vector/||q.Vector|| * sin(||q.Vector||))

        Returns:
            (1) parameter [Vector<cls_data_type> 1x4]: Output (exponential) vector as a quaternion (w, x, y, z) with the desired 
                                                       data type. 
        """

        v_norm = self.Vector.Norm() 
        if v_norm > Mathematics.CONST_EPS_64:
            return np.exp(self.__q[0]) * self.__class__(np.append(np.cos(v_norm), self.Vector.Normalize().all() * np.sin(v_norm)).tolist(), 
                                                        self.__data_type)
        else:
            return self.__class__([np.cos(v_norm), 0.0, 0.0, 0.0], self.__data_type)

    def Rotate(self, x: tp.List[cls_data_type]) -> tp.List[cls_data_type]:
        """
        Description:
            Get the rotation of a 3D vector or quaternion according to the rotation stored in the class object.

        Args:
            (1) x [Vector<cls_data_type> 1x3, 1x4]: Input vector with defined data type (Vector3_Cls 1x3, Quaternion 1x4).

        Returns:
            (1) parameter [Vector<cls_data_type> 1x3, 1x4]: Output (rotated) vector with the desired data 
                                                            type (Vector3_Cls 1x3, Quaternion 1x4).
        """

        # Get the normalized (unit) quaternion from the 
        # input quaternion.
        self.Normalize()

        if isinstance(x, self.__class__):
            return self * x.Normalize() * self.Inverse()
        elif isinstance(x, Vector3_Cls):
            # Create a quaternion form from a vector: 
            #   quaternion (w, x, y, z) = (0.0, v)
            return (self * self.__class__(np.append([0.0], x.all()).tolist(), self.__data_type) * self.Inverse()).Vector

    def Distance(self, method: str, q: tp.List[cls_data_type]) -> cls_data_type:
        """
        Description:
            Get the distance between two quaternions using the selected method.

            The equation of the distance calculation is defined as follows:
                1\ Euclidean norm
                    d(q_{0}, q_{1}) = min(||q_{0} - q_{1}||, ||q_{0} + q_{1}||)
                2\ Geodesic norm
                    d(q_{0}, q_{1}) = ln(||q_{0}^(-1) * q_{1}||)

        Args:
            (1) method [string]: The name of the method to calculate the distance 
                                 between two quaternions.
                                 Note:
                                    method = 'Euclidean' or 'Geodesic'
            (2) q [Vector<cls_data_type> 1x4]: Input scalar or quaternion (w, x, y, z).

        Returns:
            (1) parameter [cls_data_type]: Distance calculated using the selected method.

        """

        if isinstance(q, (list, np.ndarray)):
            q = self.__class__(q, self.__data_type)

        # Get the normalized (unit) quaternion from the 
        # input quaternion.
        q.Normalize(); self.Normalize()

        if method == 'Euclidean':
            q_norm_m = (self - q).Norm()
            q_norm_p = (self + q).Norm()

            return q_norm_m if q_norm_m < q_norm_p else q_norm_p 
        elif method == 'Geodesic':
            return (self.Inverse() * q).Logarithm().Norm()

    def Get_Angle_Axis(self):
        """
        Description:
            Get the Angle-Axis representation of the orientation from the Quaternion.

            Angle:
                alpha = 2.0 * acos(q.w)
            Axis: 
                v_hat = q.Vector / ||q.Vector||

        Returns:
            (1) parameter [float]: Angle of rotation.
            (2) parameter [Vector<float> 1x3]: Direction of the axis of rotation.
        """

        return (2.0 * np.arccos(self.__q[0]), self.Vector.Normalize())

    def Get_Homogeneous_Transformation_Matrix(self, method) -> tp.List[tp.List[cls_data_type]]:
        """
        Description:
            Get the homogeneous transformation matrix {T} from the Quaternion (w, x, y, z) representation of orientation.

            The shape of the matrix {T} below is based on the simplification of the expression by the chosen method.

            Note:
                For more information on simplifying the function, see:
                    ../Lib/Simplification/Quaternion.py

        Args:
            (1) method [string]: The name of the method to calculate the homogeneous transformation matrix {T}.
                                 Note:
                                    method = 'Homogeneous' or 'Inhomogeneous'.

        Returns:
            (1) parameter [Matrix<cls_data_type> 4x4]: Homogeneous transformation matrix {T}.
        """
        
        # Get the normalized (unit) quaternion from the 
        # input quaternion.
        self.Normalize()

        if method == 'Homogeneous':
            T = [[ self.__q[0]**2 + self.__q[1]**2 - self.__q[2]**2 - self.__q[3]**2,        -2.0*self.__q[0]*self.__q[3] + 2.0*self.__q[1]*self.__q[2],         2.0*self.__q[0]*self.__q[2] + 2.0*self.__q[1]*self.__q[3], 0.0],
                 [         2.0*self.__q[0]*self.__q[3] + 2.0*self.__q[1]*self.__q[2], self.__q[0]**2 - self.__q[1]**2 + self.__q[2]**2 - self.__q[3]**2,        -2.0*self.__q[0]*self.__q[1] + 2.0*self.__q[2]*self.__q[3], 0.0],
                 [        -2.0*self.__q[0]*self.__q[2] + 2.0*self.__q[1]*self.__q[3],         2.0*self.__q[0]*self.__q[1] + 2.0*self.__q[2]*self.__q[3], self.__q[0]**2 - self.__q[1]**2 - self.__q[2]**2 + self.__q[3]**2, 0.0],
                 [                                                               0.0,                                                               0.0,                                                               0.0, 1.0]]
        elif method == 'Inhomogeneous':
            T = [[            -2.0*self.__q[2]**2 - 2.0*self.__q[3]**2 + 1.0, -2.0*self.__q[0]*self.__q[3] + 2.0*self.__q[1]*self.__q[2],  2.0*self.__q[0]*self.__q[2] + 2.0*self.__q[1]*self.__q[3], 0.0],
                 [ 2.0*self.__q[0]*self.__q[3] + 2.0*self.__q[1]*self.__q[2],             -2.0*self.__q[1]**2 - 2.0*self.__q[3]**2 + 1.0, -2.0*self.__q[0]*self.__q[1] + 2.0*self.__q[2]*self.__q[3], 0.0],
                 [-2.0*self.__q[0]*self.__q[2] + 2.0*self.__q[1]*self.__q[3],  2.0*self.__q[0]*self.__q[1] + 2.0*self.__q[2]*self.__q[3],             -2.0*self.__q[1]**2 - 2.0*self.__q[2]**2 + 1.0, 0.0],
                 [                                                       0.0,                                                        0.0,                                                        0.0, 1.0]]

        return Homogeneous_Transformation_Matrix_Cls(T, np.float64)
