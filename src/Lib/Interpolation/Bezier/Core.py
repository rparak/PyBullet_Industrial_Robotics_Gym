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
# Custom Lib.::
#   ../Interpolation/Utilities
import Lib.Interpolation.Utilities as Utilities
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

class Bezier_Cls(object):
    """
    Description:
        A specific class for working with Bézier curves.

            The Bézier curve is defined as:
                (a) Explicit Form
                    B(x) = sum_{i=0}^{n} B_{i, n}(x) * P_{i},

                    where B_{i, n}(x) are the Bernstein basis polynomials of degree {n}, and P_{i} are control points.

                    Note:
                        See the Bernstein_Polynomial(i, n, x) function in ./Utilities.py for more information.

                (b) Polynomial Form
                    B(x) = sum_{j=0}^{n} x^(j) * C_{j},

                    where C_{j} is a polynomial of degree {j}.

                    C_{j} is defined as:
                        C_{j} = \prod_{m=0}^{j-1} (n - m) \sum_{i=0}^{j} ((-1)^(i + j) * P_{i}) / (i!(j - i)!)

                    Warning:
                        However, caution should be exercised as high-order curves may lack numerical stability. Note that the empty
                        product is equal to 1.

        The value of the time must be within the interval: 
            0.0 <= x <= 1.0.

        The points must be in the following form:
            P = [p_0{x, y, ..}, 
                 p_1{x, y, ..}, 
                 ...].

    Initialization of the Class:
        Args:
            (1) method [string]: The name of the method to be used to interpolate the parametric curve.
                                    Note:
                                        method = 'Explicit' or 'Polynomial'.
            (2) P [Vector<float> mxn]: Input control points to be interpolated.
                                          Note:
                                            Where m is the number of points and n is the dimension (2-D, 3-D).
            (3) N [int]: The number of interpolated points of the parametric curve.

        Example:
            Initialization:
                # Assignment of the variables.
                method = 'Explicit'; N = 100
                P = np.array([[1.00,  0.00], 
                              [2.00, -0.75], 
                              [3.00, -2.50], 
                              [3.75, -1.25], 
                              [4.00,  0.75], 
                              [5.00,  1.00]], dtype=np.float64)

                # Initialization of the class.
                Cls = Bezier_Cls(method, P, N)

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
        
    def __init__(self, method: str, P: tp.List[tp.List[float]], N: int) -> None:
        try:
            assert method in ['Explicit', 'Polynomial']

            # Convert the string to an identification number.
            #   'Explicit': 0; 'Polynomial': 1
            self.__method_id = 0 if method == 'Explicit' else 1

            # The value of the time must be within the interval: 
            #   0.0 <= x <= 1.0
            self.__x = np.linspace(Utilities.CONST_T_0, Utilities.CONST_T_1, N)

            # Initialization of other class parameters.
            #   Control Points.
            self.__P = np.array(P, dtype=np.float64)
            #   Dimension (2-D, 3-D).
            self.__dim = self.__P.shape[1]
            #   Interpolated points.
            self.__B = np.zeros((N, self.__dim), dtype=np.float64)
            #   First derivation of interpolated points.
            self.__B_dot = np.zeros((N, self.__dim), dtype=np.float64)
            #   Degree of a polynomial.
            self.__n = self.__P.shape[0] - 1

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect type of class input parameters. The calculation method must correspond to the name Explicit or Polynomial, not {method}.')

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
                                            Where m is the number of points and n is the dimension (2-D, 3-D).
        """
                
        try:
            assert P.shape[1] == self.__dim

            self.__P = np.array(P, dtype=np.float64)

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect dimensions of input control points. The point dimension must be {self.__dim} and not {P.shape[1]}.')
    
    @property
    def B(self) -> tp.List[tp.List[float]]:
        """
        Description:
           Get the interpolated points of the curve.
        
        Returns:
            (1) parameter [Vector<float> mxn]: Interpolated points.
                                                Note:
                                                    Where m is the number of points and n is the dimension (2-D, 3-D).
        """
                
        return self.__B
    
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
                
        return self.__x.shape[0]
    
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
                L(x) = \int_{0}^{t} ||B'(x)||_{2} dx.

        Returns:
            (1) parameter [float]: The arc length L(x) of the general parametric curve.
        """
                
        # Obtain the first derivative of the Bézier curve. 
        _ = self.Derivative_1st()
        
        L = 0.0
        for _, B_dot_i in enumerate(self.__B_dot):
            L += Mathematics.Euclidean_Norm(B_dot_i)

        return L / self.N
    
    def __Get_Initial_Min_Max_BB(self, P_0: tp.List[float], P_N: tp.List[float]) -> tp.Tuple[tp.List[float], 
                                                                                             tp.List[float]]:
        """
        Description:
           Obtain the minimum and maximum values in each axis from the input control points (P_0, P_N).
        
        Args:
            (1, 2) P_0, P_N [Vector<float> mxn]: First (P_0) and last (P_N) control point.
                                                    Note:
                                                        Where m is the number of points and n is the dimension (2-D, 3-D).

        Returns:
            (1) parameter [Vector<float> 1xn]: Minimum values in each axis from control points P_0 and P_N.
            (2) parameter [Vector<float> 1xn]: Maximum values in each axis from control points P_0 and P_N.
                                                Note:
                                                    Where n is the dimension (2-D, 3-D).
        """

        min = np.zeros(self.__dim, dtype=np.float64); max = min.copy()
        for i, (p_0_i, p_n_i) in enumerate(zip(P_0, P_N)):
            min[i] = Mathematics.Min([p_0_i, p_n_i])[1]
            max[i] = Mathematics.Max([p_0_i, p_n_i])[1]

        return (min, max)
    
    def __Get_B_x(self, P: tp.List[float], x: tp.List[float]) -> tp.List[float]:
        """
        Description:
            Obtain the interpolated control points with the defined time value x.
        
        Args:
            (1) P [Vector<float> mxn]: Input control points to be interpolated.
                                        Note:
                                            Where m is the number of points and n is the dimension (2-D, 3-D).
            (2) x [Vector<float> 1xk]: Defined time value x.
                                        Note:
                                            Where k is the number of values in the vector.

        Returns:
            (1) parameter [Vector<float> 1xn]: Interpolated points.
                                                Note:
                                                    Where n is the dimension (2-D, 3-D).
        """

        B = np.zeros(self.__dim, dtype=np.float64)
        for j, p_j in enumerate(P):
            B += Utilities.Bernstein_Polynomial(j, self.__n, x) * p_j

        return B

    def __C(self, j: int) -> tp.List[float]:
        """
        Description:
            Obtain a polynomial of degree {j} for each axis of point {P}.
        
        Args:
            (1) j [int]: The current degree of the polynomial.

        Returns:
            (1) parameter [Vector<float> 1xn]: Polynomial of degree {j}.
                                                Note:
                                                    Where n is the dimension (2-D, 3-D) of the point {P}.
        """
                
        eq_ls = 1.0
        for m in range(0, j):
            eq_ls *= self.__n - m

        eq_rs = 0.0
        for i in range(0, j + 1):
            eq_rs += (((-1) ** (i + j)) * self.__P[i]) / (Mathematics.Factorial(i)*Mathematics.Factorial(j - i))

        return eq_ls * eq_rs
    
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
        
            if limitation == 'Control-Points':
                min = np.zeros(self.__dim, dtype=np.float64); max = min.copy()
                for i, P_T in enumerate(self.__P.T):
                    min[i] = Mathematics.Min(P_T)[1]
                    max[i] = Mathematics.Max(P_T)[1]
            else:
                # Obtain the minimum and maximum values in each axis from 
                # the input control points (P_0, P_N).
                (min, max) = self.__Get_Initial_Min_Max_BB(self.__P[0], self.__P[-1])

                # Find the coefficients of the polynomial of degree n.
                #   Note:
                #       The coefficients are of the form x^0 + x^1 + .. + x^n.
                coeff = np.array([i*self.__C(i) for i in range(1, self.__n + 1)],
                                 dtype=np.float64).T
                
                # Calculate the roots of the parametric curve to obtain the minimum 
                # and maximum on the axis for x between the values 0.0 and 1.0.
                for i, coeff_i in enumerate(coeff):
                    if coeff_i.size != 1:
                        # Find the roots of the equation of a polynomial of degree n.
                        #   Note:
                        #       We need to invert the vector of coefficients 
                        #       and get the form x^n + .. + x^1 + x^0.
                        roots = Mathematics.Roots(coeff_i[::-1])

                        # The value of the time must be within the interval: 
                        #   0.0 <= x <= 1.0
                        x_tmp = []
                        for _, roots_i in enumerate(roots):
                            if Utilities.CONST_T_0 <= roots_i <= Utilities.CONST_T_1:
                                x_tmp.append(roots_i)
                        
                        # Remove duplicates from the vector.
                        x_tmp = np.array([*set(x_tmp)], dtype=np.float64)
                        
                        if x_tmp.size == 0:
                            continue
                        else:
                            x = x_tmp
                    else:
                        if Utilities.CONST_T_0 <= coeff_i <= Utilities.CONST_T_1:
                            x = np.array(coeff_i, dtype=np.float64)
                        else:
                            continue

                    # Find the points on the interpolated parametric Bézier curve 
                    # that correspond to time x.
                    B_i = []
                    for _, x_i in enumerate(x):
                        B_i.append(self.__Get_B_x(self.__P[:, i], np.array(x_i, dtype=np.float64)))
                    B_i = np.array(B_i, dtype=np.float64).flatten()

                    # Obtain the minimum and maximum of the parametric curve in the i-axis.
                    #   Note:
                    #       If i = 0, we get the x-axis and so on.
                    min[i] = Mathematics.Min(np.append(min[i], B_i))[1]
                    max[i] = Mathematics.Max(np.append(max[i], B_i))[1]

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

        return Utilities.RDP_Simplification(self.__B, epsilon)
    
    def Derivative_1st(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Obtain the first derivative of the Bézier curve of degree {n}.

                The first derivative of a Bézier curve of degree n is defined as:
                    (a) Explicit Form
                        B'(x) = n sum_{i=0}^{n-1} B_{i, n-1}(x) * (P_{i+1} - P_{i}).

                    (b) Polynomial Form
                        B'(x) = sum_{j=1}^{n+1} x^(j-1) * C_{j} * j.

            Note:
                The function uses both explicit and polynomial methods to obtain 
                the interpolated points.

        Returns:
            (1) parameter [Vector<float> Nxn]: Interpolated points of the first derivative of the parametric Bézier curve.
                                                Note:
                                                    Where N is the number of points and n is the dimension (2-D, 3-D).
        """
                
        self.__B_dot = np.zeros(self.__B_dot.shape, dtype=self.__B_dot.dtype)

        if self.__method_id == 0:
            # Explicit form.
            #   Note:
            #       De Casteljau's algorithm.
            n = self.__n - 1
            for i, (p_i, p_ii) in enumerate(zip(self.__P, self.__P[1:])):
                for j, (p_ij, p_iij) in enumerate(zip(p_i, p_ii)):
                    self.__B_dot[:, j] += Utilities.Bernstein_Polynomial(i, n, self.__x) * (p_iij - p_ij)
            self.__B_dot = self.__n * self.__B_dot
        elif self.__method_id == 1:
            # Polynomial form.
            for j in range(1, self.__n + 1):
                for i, C_j in enumerate(self.__C(j)):
                    self.__B_dot[:, i] += (self.__x ** (j - 1)) * C_j * j

        return self.__B_dot
    
    def Interpolate(self) -> tp.List[tp.List[float]]:  
        """
        Description:
            Obtain the interpolated points of the parametric Bézier curve of degree {n}.

            Note:
                The function uses both explicit and polynomial methods to obtain 
                the interpolated points.

        Returns:
            (1) parameter [Vector<float> Nxn]: Interpolated points of the parametric Bézier curve.
                                                Note:
                                                    Where N is the number of points and n is the dimension (2-D, 3-D).
        """
                  
        self.__B = np.zeros(self.__B.shape, dtype=self.__B.dtype)
        if self.__method_id == 0:
            # Explicit form.
            #   Note:
            #       De Casteljau's algorithm.
            for i, p_i in enumerate(self.__P):
                for j, p_ij in enumerate(p_i):
                    self.__B[:, j] += Utilities.Bernstein_Polynomial(i, self.__n, self.__x) * p_ij
        elif self.__method_id == 1:
            # Polynomial form.
            for j in range(0, self.__n + 1):
                for i, C_j in enumerate(self.__C(j)):
                    self.__B[:, i] += (self.__x ** j) * C_j
                    
        return self.__B
