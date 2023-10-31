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
# Custom Lib.:
#   ../Interpolation/Utilities
import Lib.Interpolation.Utilities as Utilities
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

class B_Spline_Cls(object):
    """
    Description:
        A specific class for working with B-Spline curves.

            The B-Spline curve is defined as:
                S(x) = sum_{i=0}^{n} B_{i, n}(x) * P_{i},

                where B_{i, n}(x) are the i-th B-spline basis functions of degree {n}, and P_{i} are control points.

                Note:
                    See the Basic_Function(i, n, t, x) function in ./Utilities.py for more information.

        The value of the time must be within the interval of the knot vector: 
            t[0] <= x <= t[-1].

        The points must be in the following form:
            P = [p_0{x, y, ..}, 
                 p_1{x, y, ..}, 
                 ...].

    Initialization of the Class:
        Args:
            (1) n [int]: Degree of a polynomial.
            (2) method [string]: The method to be used to select the parameters of the knot vector. 
                                    Note: 
                                        method = 'Uniformly-Spaced', 'Chord-Length' or 'Centripetal'.
            (3) P [Vector<float> mxn]: Input control points to be interpolated.
                                          Note:
                                            Where m is the number of points and n is the dimension (2-D, 3-D).
            (4) N [int]: The number of interpolated points of the parametric curve.

        Example:
            Initialization:
                # Assignment of the variables.
                n = 3; method = 'Chord-Length'; N = 100
                P = np.array([[1.00,  0.00], 
                              [2.00, -0.75], 
                              [3.00, -2.50], 
                              [3.75, -1.25], 
                              [4.00,  0.75], 
                              [5.00,  1.00]], dtype=np.float64)

                # Initialization of the class.
                Cls = B_Spline_Cls(n, method, P, N)

            Features:
                # Properties of the class.
                Cls.P; Cls.x, Cls.N
                ...
                Cls.dim

                # Functions of the class.
                Cls.Get_Arc_Length()
                ...
                Cls.Interpolate()
    """
        
    # Create a global data type for the class.
    cls_data_type = tp.TypeVar('cls_data_type')

    def __init__(self, n: int, method: str, P: tp.List[tp.List[float]], N: int) -> None:
        try:
            assert n < P.shape[0]

            # The method to be used to select the parameters of the knot vector.
            self.__method = method

            # Generate a normalized vector of knots from the selected parameters 
            # using the chosen method.
            self.__t = Utilities.Generate_Knot_Vector(n, P, self.__method)

            # The value of the time must be within the interval of the knot vector: 
            #   t[0] <= x <= t[-1]
            self.__x = np.linspace(self.__t[0], self.__t[-1], N)

            # Initialization of other class parameters.
            #   Control Points.
            self.__P = np.array(P, dtype=np.float64)
            #   Dimension (2-D, 3-D).
            self.__dim = self.__P.shape[1]
            #   Interpolated points.
            self.__S = np.zeros((N, self.__dim), dtype=np.float64)
            #   First derivation of interpolated points.
            self.__S_dot = np.zeros((N, self.__dim), dtype=np.float64)
            #   Degree of a polynomial.
            self.__n = n
            #   The number of interpolated points of the parametric curve.
            self.__N = N

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of class input parameters.')
            # Error information.
            if n >= P.shape[0]:
                print(f'[ERROR] The degree (n = {n}) of the B-spline must be less than the number of input control points (N = {P.shape[0]}).')
                
    @property
    def n(self) -> int:
        """
        Description:
           Get the degree of a polynomial.
        
        Returns:
            (1) parameter [int]: Degree of a polynomial.
        """
                
        return self.__n

    @property
    def P(self) -> tp.List[tp.List[float]]:
        """
        Description:
           Get the control points of the curve.
        
        Returns:
            (1) parameter [Vector<float> mxn]: Control points.
                                                Note:
                                                    Where m is the number of points and n is the dimension (2-D, 3-D).
        """
                
        return self.__P
    
    @P.setter
    def P(self, P: tp.List[tp.List[float]]) -> None:
        """
        Description:
           Set the new control points of the curve.
        
        Args:
            (1) P [Vector<float> mxn]: Control points.
                                        Note:
                                            Where m is the number of points and n is the dimension (2-D, 3-D)
        """
                
        try:
            assert P.shape[1] == self.__dim

            self.__P = np.array(P, dtype=np.float64)

            # Generate a normalized vector of knots from the selected parameters 
            # using the chosen method.
            self.__t = Utilities.Generate_Knot_Vector(self.__n, P, self.__method)

            # The value of the time must be within the interval of the knot vector: 
            #   t[0] <= x <= t[-1]
            self.__x = np.linspace(self.__t[0], self.__t[-1], self.__N)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect dimensions of input control points. The point dimension must be {self.__dim} and not {P.shape[1]}.')
    
    @property
    def S(self) -> tp.List[tp.List[float]]:
        """
        Description:
           Get the interpolated points of the curve.
        
        Returns:
            (1) parameter [Vector<float> mxn]: Interpolated points.
                                                Note:
                                                    Where m is the number of points and n is the dimension (2-D, 3-D).
        """
                
        return self.__S
    
    @property
    def t(self) -> tp.List[float]:
        """
        Description:
           Get the normalized vector of knots.
        
        Returns:
            (1) parameter [Vector<float> 1xm]: Normalized knot vector as a non-decreasing sequence of real numbers.
                                                Note:
                                                    Where m is the number of generated knots defined by the formula:
                                                        N (number of control points) + n (degree) + 1.
        """
                
        return self.__t
    
    @property
    def x(self) -> tp.List[float]:
        """
        Description:
           Get the time as an interval of values from 0 to 1.
        
        Returns:
            (1) parameter [Vector<float> 1xn]: Time.
                                                Note:
                                                    Where n is the number of points.
        """
                
        return self.__x
    
    @property
    def N(self) -> int:
        """
        Description:
           Get the number of interpolated points of the parametric curve.
        
        Returns:
            (1) parameter [int]: Number of interpolated points. 
        """
                
        return self.__N
    
    @property
    def dim(self) -> int:
        """
        Description:
           Get the dimension (2-D, 3-D) of the control/interpolated points.
        
        Returns:
            (1) parameter [int]: The dimension of the points at which the interpolation is performed.
        """
                
        return self.__dim
    
    def Get_Arc_Length(self) -> float:
        """
        Description:
            Obtain the arc length L(x) of the general parametric curve.

            The arc length L(x) is defined by:
                L(x) = \int_{0}^{x} ||B'(x)||_{2} dx.

        Returns:
            (1) parameter [float]: The arc length L(x) of the general parametric curve.
        """
                
        # Obtain the first derivative of the B-Spline curve.  
        _ = self.Derivative_1st()

        L = 0.0
        for _, S_dot_i in enumerate(self.__S_dot):
            L += Mathematics.Euclidean_Norm(S_dot_i)

        return L / (self.N)

    def Optimize_Control_Points(self, N: int) -> cls_data_type:
        """
        Description:
            Obtain optimized control points from a set of control points on a B-Spline curve using the Least-Squares method.

                The fitted B-spline curve is formally presented in equation:
                    S(x) = sum_{i=0}^{n} B_{i, n}(x) * Q_{i},
                
                but the control points Q_{i} are unknown quantities to be determined later.

                From the link below we can get the equation to calculate Q:
                    Q = (A^T * A)^(-1) * A^T * P,

                wher Q are optimized control points, P are the input control points on the B-Spline 
                curve, and A are the i-th B-spline basis functions of degree {n}.
                
            Reference:
                https://www.geometrictools.com/Documentation/BSplineCurveLeastSquaresFit.pdf

        Args:
            (1) N [int]: The resulting number of optimized control points.

        Returns:
            (1) parameter [B_Spline_Cls(object)]: Self-class (B-Spline) with optimized control points as input. The other class parameters 
                                                  remain unchanged.
        """

        try:
            assert N < self.__P.shape[0] and N > self.__n and N != 1

            # Generate a normalized vector of knots from the selected parameters
            # using the Uniformly-Spaced method.
            t = Utilities.Generate_Knot_Vector(self.__n, np.zeros((N, 1), dtype=self.__P.dtype), 
                                              'Uniformly-Spaced')
            
            # The value of the time must be within the interval of the knot vector: 
            #   t[0] <= x <= t[-1]
            x = np.linspace(self.__t[0], self.__t[-1], self.__P.shape[0])
            
            """
            Description:
                Least-Squares Fitting.
            """
            A = np.zeros((self.__P.shape[0], N), dtype=self.__P.dtype)
            for i in range(self.__P.shape[0]):
                for j in range(N):
                    A[i, j] = Utilities.Basic_Function(j, self.__n, t, x[i])

            Q = (np.linalg.inv(A.T @ A) @ A.T) @ self.__P
            Q[0] = self.__P[0]; Q[-1] = self.__P[-1]

            return self.__class__(self.__n, self.__method, Q, self.N)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect type of class input parameters.')
            # Error information.
            if N >= self.__P.shape[0]:
                print(f'[ERROR] The number of optimized control points (N = {N}) must be less than the number of input control points (N_in = {self.__P.shape[0]}).')
            if N <= self.__n:
                print(f'[ERROR] The number of optimized control points (N = {N}) must be greater than the degree (n = {self.__n}) of the B-spline.')
            if N == 1:
                print('[ERROR] The number of optimized control points cannot be equal to 1.')
            
    def Get_Bounding_Box_Parameters(self, limitation: str) -> tp.Tuple[tp.List[float]]:
        """
        Description:
            Obtain the bounding parameters (min, max) of the general parametric curve (interpolated 
            points) as well as the control points.
        
        Args:
            (1) limitation [string]: The limitation to be used to describe the bounding box.
                                        Note:
                                            limitation = 'Control-Points'
                                                - The result depends on the parameters (min, max) of the control points.
                                            limitation = 'Interpolated-Points'
                                                - The result depends on the parameters (min, max) of the general parametric 
                                                  curve (interpolated points).

        Returns:
            (1) parameter [Dictionary {'x_min': int, 'y_min': int, etc.}]: Bounding box parameters (min, max) defined by the limitation 
                                                                           from the arguments of the function.
                                                                            Note:
                                                                                The number of values in both parameters min and max depends 
                                                                                on the dimension of the points.
        """

        try:
            assert limitation in ['Control-Points', 'Interpolated-Points']

            min = np.zeros(self.__dim, dtype=np.float64); max = min.copy()
            if limitation == 'Control-Points':
                for i, P_T in enumerate(self.__P.T):
                    min[i] = Mathematics.Min(P_T)[1]
                    max[i] = Mathematics.Max(P_T)[1]
            else:
                for i, S_T in enumerate(self.__S.T):
                    min[i] = Mathematics.Min(S_T)[1]
                    max[i] = Mathematics.Max(S_T)[1]

            if self.__dim == 2:
                return {'x_min': min[0], 'x_max': max[0], 
                        'y_min': min[1], 'y_max': max[1]}
            else:
                return {'x_min': min[0], 'x_max': max[0], 
                        'y_min': min[1], 'y_max': max[1],
                        'z_min': min[2], 'z_max': max[2]}
        
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect type of function input parameters. The limitation must correspond to the name Control-Points or Interpolated-Points, not {limitation}.')

    def Reduce_Interpolated_Points(self, epsilon: float) -> tp.List[float]:
        """
        Description:
            A function to simplify (reduce) a given array of interpolated points using 
            the Ramer-Douglas-Peucker algorithm.
        
        Args:
            (1) epsilon [float]: The coefficient determines the similarity between the original a
                                 and the approximated curve. 
                                    Note: 
                                        epsilon > 0.0.

        Returns:
            (1) parameter [Vector<float> ]: Simplified (reduced) vector of interpolated points {B}.
        """

        return Utilities.RDP_Simplification(self.__S, epsilon)

    def Derivative_1st(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Obtain the first derivative of the B-Spline curve of degree {n} using De Boor's algorithm.

                The first derivative of a B-Spline curve of degree n is defined as:
                    S'(x) = sum_{i=0}^{n-1} B_{i, n - 1}(x) * Q_{i},
                
                    where Q_{i} are defined as follows:
                        Q_{i} = (n / (t_dot[j + n] - t_dot[j])) * (P_{i+1} - P_{i}),

                        where t_dot is described as the derivative of the vector of knots with the first 
                        and last knot removed.

                    Therefore, the derivative of the B-spline curve is another B-spline curve of degree n - 1 on the original 
                    knot vector with a new set of n control points Q_{0}, Q{1}, .. , Q_{n-1}.

        Returns:
            (1) parameter [Vector<float> Nxn]: Interpolated points of the first derivative of the parametric B-Spline curve.
                                                Note:
                                                    Where N is the number of points and n is the dimension (2-D, 3-D).
        """

        self.__S_dot = np.zeros(self.__S_dot.shape, dtype=self.__S_dot.dtype); t_dot = self.__t[1:-1]
        for i, x_i in enumerate(self.__x):
            for j, (p_i, p_ii) in enumerate(zip(self.__P, self.__P[1:])):   
                Q_i = (self.__n / (t_dot[j + self.__n] - t_dot[j])) * (p_ii - p_i)
                self.__S_dot[i, :] += Utilities.Basic_Function(j, self.__n - 1, t_dot, x_i) * Q_i

        return self.__S_dot
    
    def Interpolate(self) -> tp.List[tp.List[float]]:  
        """
        Description:
            Obtain the interpolated points of the parametric B-Spline curve of degree {n} using De Boor's algorithm.

        Returns:
            (1) parameter [Vector<float> Nxn]: Interpolated points of the parametric B-Spline curve.
                                                Note:
                                                    Where N is the number of points and n is the dimension (2-D, 3-D).
        """
                  
        self.__S = np.zeros(self.__S.shape, dtype=self.__S.dtype)
        for i, x_i in enumerate(self.__x):
            for j, p_i in enumerate(self.__P):
                self.__S[i, :] += Utilities.Basic_Function(j, self.__n, self.__t, x_i) * p_i

        return self.__S
