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
File Name: ../RoLE/Interpolation/Utilities.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics

"""
Description:
    Initialization of constants.
"""
# Time x ∈ [0: The starting value of the sequence, 
#           1: The end value of the sequence] {0.0 <= x <= 1.0}
CONST_T_0 = 0.0
CONST_T_1 = 1.0

def Lerp(method: str, p_0: tp.List[float], p_1: tp.List[float], x: float) -> tp.List[float]:
    """
    Description:
        Linear interpolation (Lerp) is a method of curve generating using linear polynomials to construct new data 
        points over the range of a discrete set of known data points. 

        The equation of the Lerp function is defined as follows:
            1\ Explicit Method:
                B(x) = (1 - t) * p_{0} + x * p_{1},
            
            2\ Polynomial Method:
                B(x) = p_{0} + x * (p_{1} - p_{0}),

            where x can take values 0.0 <= x <= 1.0.
    
    Args:
        (1) method [string]: The name of the method to calculate the {Lerp} function.
                             Note:
                                method = 'Explicit' or 'Polynomial'
        (2) p_0, p_1 [Vector<float> 1xn]: Input points to be interpolated.
                                          Note:
                                            Where n is the number of dimensions of the point.
        (3) x [float]: Time x (0.0 <= x <= 1.0).
        
    Returns: 
        (1) parameter [Vector<float> 1xn]: Interpolated point at time {t}.
                                           Note:
                                            Where n is the number of dimensions of the point.
    """

    try:
        assert method in ['Explicit', 'Polynomial']

        # The time value must be within the interval: 0.0 <= x <= 1.0
        x = Mathematics.Clamp(x, CONST_T_0, CONST_T_1)

        if method == 'Explicit':
            # Equation:
            #   B(x) = (1 - x) * p_{0} + x * p_{1}
            return (1 - x) * p_0 + x * p_1
        elif method == 'Polynomial':
            # Equation:
            #   B(x) = p_{0} + x * (p_{1} - p_{0})
            return p_0 + x * (p_1 - p_0)
        
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] Incorrect type of function input parameters. The calculation method must correspond to the name Explicit or Polynomial, not {method}.')

def Slerp(method: str, q_0: tp.List[float], q_1: tp.List[float], x: float) -> tp.List[float]:
    """
    Description:
        Performs a spherical linear interpolation (Slerp) between the given quaternions 
        and stores the result in this quaternion.

        The equation of the Slerp function is defined as follows:
            1\ Geometric:
                B(x) = (sin[(1 - x) * theta] / sin(theta)) * q_{0} + (sin[x * theta] / sin(theta)) * q_{1},
            2\ Quaternion:
                B(x) = (q_{1} * q_{0}^(-1))^(x) * q_0,

            where x can take values 0.0 <= x <= 1.0.

    Args:
        (1) method [string]: The name of the method to calculate the {Slerp} function.
                             Note:
                                method = 'Geometric' or 'Quaternion'
        (2) q_0, q_1 [Vector<float> 1x4]: Input quaternions to be interpolated.
        (3) x [float]: Time x (0.0 <= x <= 1.0).

    Returns: 
        (1) parameter [Vector<float> 1x4]: Interpolated quaternion at time {t}.                  
    """

    try:
        assert method in ['Geometric', 'Quaternion']
    
        # The time value must be within the interval: 0.0 <= x <= 1.0
        x = Mathematics.Clamp(x, CONST_T_0, CONST_T_1)

        if isinstance(q_0, Transformation.Quaternion_Cls) and isinstance(q_1, Transformation.Quaternion_Cls):
            q_0 = q_0.Normalize()
            q_1 = q_1.Normalize()
        else:
            q_0 = Transformation.Quaternion_Cls(q_0, np.float64).Normalize()
            q_1 = Transformation.Quaternion_Cls(q_1, np.float64).Normalize()

        if method == 'Geometric':
            # Calculate angle between quaternions (q_0, q_1).
            q_01_angle = q_0.Dot(q_1)

            if q_01_angle < 0.0:
                q_0 = -q_0; q_01_angle = -q_01_angle

            if q_01_angle > 0.9995:
                # If the input quaternions are too close, perform a linear 
                # interpolation (Lerp):
                #   Lerp (polynomial form): q_{0} + x * (q_{1} - q_{0})
                return Lerp('Polynomial', q_0, q_1, x)

            # Auxiliary expression for the final equation.
            theta = np.arccos(q_01_angle); sin_theta = np.sin(theta)

            return (np.sin((1 - x) * theta) / sin_theta) * q_0 + (np.sin(x * theta) / sin_theta) * q_1
        elif method == 'Quaternion':
            return (q_1 * q_0.Inverse())**x * q_0
    
    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] Incorrect type of function input parameters. The calculation method must correspond to the name Geometric or Quaternion, not {method}.')
        
def Bernstein_Polynomial(i: int, n: int, x: tp.List[float]) -> tp.List[float]:
    """
    Description:
        Bernstein polynomials form the basis of the Bézier elements used in isogeometric analysis.

        For a given {n >= 0}, define the {n + 1} Bernstein basis polynomials of degree {n} on [0,1] as:
            B_{i, n}(x) = (n i) * (1 - x)^(n - i) * (x^i), i = 0,..,n,

            where (n i) is a binomial coefficient.

        There are four of them for n = 3, for example:
            B_{0, 3} = (1 - x)^3
            B_{1, 3} = 3 * x * (1 - x)^2
            B_{2, 3} = 3 * (x^2) * (1 - x)
            B_{3, 3} = x^3

    Args:
        (1) i [int]: Iteration.
        (2) n [int]: Degree of a polynomial.
        (3) x [Vector<float> 1xk]: Time x ∈ [0: The starting value of the sequence, 1: The end value of the sequence].
                                   {0.0 <= x <= 1.0}
                                   Note:
                                    Where k is the number of elements of the time vector.
                             
    Returns:
        (1) parameter [Vector<float> 1xk]: A Bernstein polynomial of degree n.
                                           Note:
                                            Where k is the number of elements of the time vector.
    """

    try:
        assert n >= Mathematics.CONST_NULL
  
        # Bernstein basis polynomials b_{i, n}(x).
        return Mathematics.Binomial_Coefficient(n, i) * (1 - x) ** (n - i) * (x ** i)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] The input condition for the polynomial calculation is not satisfied. The degree of n ({n}) must be greater than or equal to {Mathematics.CONST_NULL}.')


def Basic_Function(i: int, n: int, t: tp.List[float], x: float) -> float:
    """
    Description:
        Get the 'B_in' as a recursively defined i-th B-spline basis functions of degree n.

        The i-th basis function of degree {n} with {t} knots is defined as follows:

            B_{i, n}(x) = ((x - t_{i})/(t_{i + n} - t_{i})) *  B_{i, n - 1}(x) + 
                          ((t_{i + n + 1} - x)/(t_{i + n + 1} - t_{i + 1})) *  B_{i + 1, n - 1}(x),

        for all real numbers x, where:
        
            B_{i, 0}(x) -> 1, x -> [t_{i}, t_{i + 1})
                        -> 0, otherwise.    

    Args:
        (1) i [int]: Iteration.
        (2) n [int]: Degree.
        (3) t [Vector<float> 1xm]: Normalized knot vector as a non-decreasing sequence of real numbers.
                                    Note:
                                        Where m is the number of generated knots defined by the formula:
                                            N (number of control points) + n (degree) + 1.
        (4) x [float]: The time parameter in the current episode.
    
    Returns:
        (1) parameter [float]: The result 'B_in' of the basis function calculated from the input parameters.
    """

    if n == 0:
        if x == 1:
            return 1.0 if t[i] <= x <= t[i + 1] else 0.0
        else:
            return 1.0 if t[i] <= x < t[i + 1] else 0.0
    else:
        denominator_1 = t[i + n] - t[i]
        denominator_2 = t[i + n + 1] - t[i + 1]
        B_in_1 = 0.0; B_in_2 = 0.0

        if denominator_1 != 0:
            B_in_1 = ((x - t[i]) / denominator_1) * Basic_Function(i, n - 1, t, x)

        if denominator_2 != 0:
            B_in_2 = ((t[i + n + 1] - x) / denominator_2) * Basic_Function(i + 1, n - 1, t, x)

        return B_in_1 + B_in_2

def Simple_Simplification(P: tp.List[float], s_f: int) -> tp.List[float]:
    """
    Description:
        A function to simplify (reduce) the point vector by a simplification factor s_{f}. The first and end point are unchanged, the others 
        depend on the simplification factor.

        The points must be in the following form:
            P = [p_0{x, y, ..}, 
                 p_1{x, y, ..}, 
                 ...].

        Example:
            Input Points: 
                P = [1.0, 1.0], [1.25, 2.0], [1.75, 2.0], [2.0, 1.0], [1.0, -1.0], [1.25, -2.0], [1.75, -2.0], [2.0, -1.0]
            Number of points: 
                n = len(P) = 8
            Simplification Factor:
                1\ Example:
                    s_f = 1
                    P_new = [1.0, 1.0], [1.25, 2.0], [1.75, 2.0], [2.0, 1.0], [1.0, -1.0], [1.25, -2.0], [1.75, -2.0], [2.0, -1.0]
                    n = 8
                2\ Example:
                    simplification_factor = 2
                    P_new = [1.0, 1.0], [None], [1.75, 2.0], [None], [1.0, -1.0], [None], [1.75, -2.0], [2.0, -1.0] 
                    P_new = [1.0, 1.0], [1.75, 2.0], [1.0, -1.0], [1.75, -2.0], [2.0, -1.0]
                    n = 5

    Args:
        (1) P [Vector<float> mxn]: Input points to be simplified.
                                   Note:
                                    Where m is the number of points and n is the dimension (2-D, 3-D).
        (2) s_f [int]: The coefficient determines the simplification of the point vector.

    Returns:
        (1) parameter [Vector<float> mxn]: Simplified vector of points {P}.
                                             Note:
                                                Where m is the number of points after simplification 
                                                and n is the dimension (2-D, 3-D).
    """

    P_aux = []
    P_aux.append(P[0])
    
    for i in range(1, P.shape[0] - 1):
        if i % s_f == 0:
            P_aux.append(P[i])

    if (P_aux[-1] == P[-1]).all() != True:
        P_aux.append(P[-1])

    return P_aux

def RDP_Simplification(P: tp.List[float], epsilon: float) -> tp.List[float]:
    """
    Description:
        A function to simplify (reduce) a given array of points using the Ramer-Douglas-Peucker algorithm.

        Note:
            The Ramer-Douglas-Peucker (RDP) is an algorithm for reducing the number of points on a curve, that 
            is approximated by a series of points.
    
        The points must be in the following form:
            P = [p_0{x, y, ..}, 
                 p_1{x, y, ..}, 
                 ...].

    Args:
        (1) P [Vector<float> mxn]: Input points to be simplified.
                                   Note:
                                    Where m is the number of points and n is the dimension (2-D, 3-D).
        (2) epsilon [float]: The coefficient determines the similarity between the original a
                             and the approximated curve. 
                             Note: 
                                epsilon > 0.0.

    Returns:
        (1) parameter [Vector<float> mxn]: Simplified vector of points {P}.
                                             Note:
                                                Where m is the number of points after simplification 
                                                and n is the dimension (2-D, 3-D).
    """

    try:
        assert epsilon > Mathematics.CONST_NULL

        d_max = 0.0; index = 0
        # Find the point with the maximum perpendicular distance.
        for i in range(1, P.shape[0]):
            d = Mathematics.Perpendicular_Distance(P[i], P[0], P[-1])
            if d > d_max:
                d_max = d; index = i

        """
        Description:
            If the maximum perpendicular distance is greater than epsilon, then recursively 
            simplify.
        """
        if d_max > epsilon:
            rdp_0 = RDP_Simplification(P[:(index + 1)], epsilon)
            rdp_1 = RDP_Simplification(P[index:], epsilon)

            return np.vstack((rdp_0[:-1], rdp_1))
        else:
            return np.vstack((P[0], P[-1]))

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] The epsilon ({epsilon}) coefficient must be greater than zero.')

def __Uniformly_Spaced(P: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to select parameters using the Uniformly-Spaced method.

        Reference:
            https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-uniform.html

    Args:
        (1) P [Vector<float> nxm]: Input points.
                                    Note:
                                        Where n is the number of dimensions of the point, and m is the number of points.

    Returns:
        (1) parameter [Vector<float> 1xn]: Selected parameters to be used to generate the vector of knots.
                                            Note:
                                                Where n is the number of dimensions of the point.
    """
        
    return np.linspace(0.0, 1.0, P.shape[0])

def __Chord_Length(P: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to select parameters using the Chord-Length method.

        Reference:
            https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-chord-length.html

    Args:
        (1) P [Vector<float> nxm]: Input points.
                                    Note:
                                    Where n is the number of dimensions of the point, and m is the number of points.

    Returns:
        (1) parameter [Vector<float> 1xn]: Selected parameters to be used to generate the vector of knots.
                                            Note:
                                                Where n is the number of dimensions of the point.
    """

    # Express the number of control points.
    N = P.shape[0]
    
    t_k = np.zeros(N); L = 0.0
    for i in range(1, N):
        L_k = Mathematics.Euclidean_Norm(P[i] - P[i-1])
        t_k[i] = t_k[i-1] + L_k
        L += L_k

    return t_k / L

def __Centripetal(P: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to select parameters using the Centripetal method.

        Reference:
            https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-centripetal.html
    
    Args:
        (1) P [Vector<float> nxm]: Input points.
                                    Note:
                                    Where n is the number of dimensions of the point, and m is the number of points.

    Returns:
        (1) parameter [Vector<float> 1xn]: Selected parameters to be used to generate the vector of knots.
                                            Note:
                                                Where n is the number of dimensions of the point.
    """
        
    # Express the number of control points.
    N = P.shape[0]
    
    t_k = np.zeros(N); L = 0.0
    for i in range(1, N):
        L_k = Mathematics.Euclidean_Norm(P[i] - P[i-1])
        t_k[i] = t_k[i-1] + L_k ** 0.5
        L += L_k ** 0.5

    return t_k / L

def __Knot_Parameter_Selection(P: tp.List[float], method: str) -> tp.List[float]:
    """
    Description:
        A function to select the parameters of the knot vector using an individual calculation method.

    Args:
        (1) P [Vector<float> nxm]: Input points.
                                   Note:
                                    Where n is the number of dimensions of the point, and m is the number of points.
        (2) method [string]: The method to be used to select the parameters. Possible string values can be: 
                             'Uniformly-Spaced', 'Chord-Length', 'Centripetal'.

    Returns:
        (1) parameter [Vector<float> 1xn]: Selected parameters to be used to generate the vector of knots.
                                            Note:
                                                Where n is the number of dimensions of the point.
    """
        
    return {
        'Uniformly-Spaced': lambda x: __Uniformly_Spaced(x),
        'Chord-Length': lambda x: __Chord_Length(x),
        'Centripetal': lambda x: __Centripetal(x)
    }[method](P)

def Generate_Knot_Vector(n: int, P: tp.List[float], method: str) -> tp.List[float]:
    """
    Description:
        A function to generate a normalized vector of knots from selected parameters using an individual 
        selection method (Uniformly-Spaced, Chord-Length, Centripetal).

        Reference:
            https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/INT-APP/PARA-knot-generation.html

    Args:
        (1) n [int]: Degree.
        (2) P [Vector<float> nxm]: Input points.
                                   Note:
                                    Where n is the number of dimensions of the point, and m is the number of points.
        (2) method [string]: The method to be used to select the parameters. Possible string values can be: 
                             'Uniformly-Spaced', 'Chord-Length', 'Centripetal'.

    Returns:
        (1) parameter [Vector<float> 1xm]: Normalized knot vector as a non-decreasing sequence of real numbers.
                                            Note:
                                                Where m is the number of generated knots defined by the formula:
                                                    N (number of control points) + n (degree) + 1.
    """
        
    try:
        assert method in ['Uniformly-Spaced', 'Chord-Length', 'Centripetal']
        
        # Express the number of control points.
        N = P.shape[0]

        # Express the number of knots to be generated.
        t_n = N + n + 1
        
        # Select the parameters of the knot vector.
        t_param = __Knot_Parameter_Selection(P, method)

        # Generate a normalized vector of knots from the selected parameters.
        t = np.zeros(t_n)
        for i in range(t_n):
            if i < n + 1:
                t[i] = 0.0
            elif i >= N:
                t[i] = 1.0
            else:
                if n > 0:
                    t[i] = np.sum(t_param[i - n:i]) / n

        return t

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] Incorrect type of function input parameters. The generation method must correspond the name given in the function description.')
