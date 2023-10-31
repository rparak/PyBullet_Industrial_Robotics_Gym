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
File Name: Mathematics.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp

"""
Description:
    Initialization of constants.
"""
CONST_NULL = 0.0
# The difference between 1.0 and the next smallest 
# representable float larger than 1.0.
#   Machine epsilon (IEEE-754 standard): b ** (-(p - 1))
#       binary32: b (Base) = 2, p (Precision) = 24 
#       binary64: b (Base) = 2, p (Precision) = 53 
CONST_EPS_32 = 1.19e-07
CONST_EPS_64 = 2.22e-16
# Mathematical constants
CONST_MATH_PI = 3.141592653589793
CONST_MATH_HALF_PI = 1.5707963267948966

def Radian_To_Degree(x: tp.Union[float, tp.List[float]]) -> tp.Union[float, tp.List[float]]:
    """
    Description:
        Functions for converting angles from radians to degrees.

    Args:
        (1) x [float or Vector<float>]: Angle in radians.

    Returns:
        (1) parameter [float or Vector<float>]: Angle in degrees.
    """

    try:
        assert isinstance(x, float) or isinstance(x, np.ndarray)
        
        return x * (180.0 / CONST_MATH_PI)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] Incorrect type of input parameters. Input parameter x must be of type float or np.ndarray.')

def Degree_To_Radian(x: tp.Union[float, tp.List[float]]) -> tp.Union[float, tp.List[float]]:
    """
    Description:
        Functions for converting angles from degrees to radians.

    Args:
        (1) x [float or Vector<float>]: Angle in degrees.

    Returns:
        (1) parameter [float or Vector<float>]: Angle in radians.
    """

    try:
        assert isinstance(x, float) or isinstance(x, np.ndarray)
        
        return x * (CONST_MATH_PI / 180.0)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print(f'[ERROR] Incorrect type of input parameters. Input parameter x must be of type float or np.ndarray.')

def Sign(x: float) -> float:
    """
    Description:
        Function to indicate the sign of a number.

        Note:
            The signum function obtains the sign of a real number by mapping 
            a set of real numbers to a set of three real numbers {-1.0, 0.0, 1.0}.

    Args:
        (1) x [float]: Input real number.

    Returns:
        (1) parameter [float]: Signum of the input number.
    """

    return [np.abs(x) / x if x != 0.0 else 0.0]

def Euclidean_Norm(x: tp.List[float]) -> float:
    """
    Description:
        The square root of the sum of the squares of the x-coordinates.

        Equation: 
            ||x||_{2} = sqrt(x_{1}^2 + ... + x_{n}^2)

    Args:
        (1) x [Vector<float>]: Input coordinates.

    Returns:
        (1) parameter [float]: Ordinary distance from the origin to the point {x} a consequence of 
                               the Pythagorean theorem.
    """
    
    x_res = np.array(x).flatten()
    if x_res.size == 1:
        return (x ** 2) ** (0.5)
    else:
        x_res_norm = 0.0
        for _, x_res_i in enumerate(x_res):
            x_res_norm += x_res_i ** 2

        return x_res_norm ** (0.5)

def Max(x: tp.List[float]) -> tp.Tuple[int, float]:
    """
    Description:
        Function to return the maximum number from the list.
            
    Args:
        (1) x [Vector<float>]: Input list for determining the value.

    Returns:
        (1) parameter [int]: Index of the maximum value in the list.
        (2) parameter [float]: The maximum value of the list.
    """

    max_x = x[0]; idx = 0
    for i, x_i in enumerate(x[1:]):
        if x_i > max_x:
            max_x = x_i
            idx = i + 1

    return (idx, max_x)
 
def Min(x: tp.List[float]) -> tp.Tuple[int, float]:
    """
    Description:
        Function to return the minimum number from the list.
            
    Args:
        (1) in_list [Vector<float>]: Input list for determining the value.

    Returns:
        (1) parameter [int]: Index of the minimum value in the list.
        (2) parameter [float]: The minimum value of the list.
    """

    min_x = x[0]; idx = 0
    for i, x_i in enumerate(x[1:]):
        if x_i < min_x:
            min_x = x_i
            idx = i + 1

    return (idx, min_x)

def Clamp(x: float, a: float, b: float) -> float:
    """
    Description:
        Clamp (limit) values with a defined interval. For a given interval, values outside the interval 
        are clipped to the edges of the interval.

        Equation:
            Clamp(x) = max(a, min(x, b)) ∈ [a, b]

            Another variant (slower):
                Mathematics.Max([a, Mathematics.Min([x, b])[0]])[0]
            
    Args:
        (1) x [float]: The value to be clamped (preferred value).
        (2) a [float]: Maximum allowed value (lower bound).
        (3) b [float]: Minimum allowed value (upper bound).

    Returns:
        (1) parameter [float]: The value clamped to the range of {a: max} and {b: min}.
    """

    return a if x < a else b if x > b else x

def Cross(a: float, b: float) -> tp.List[float]:
    """
    Description:
        Get the cross product of two vectors.

        Equation:
            3D:
                The three scalar components of the resulting vector s = s_{1}i + s_{2}j + s_{3}k = a x b 
                are as follows:

                s_{1} = a_{2}*b_{3} - a_{3}*b_{2}
                s_{2} = a_{3}*b_{1} - a_{1}*b_{3}
                s_{3} = a_{1}*b_{2} - a_{2}*b_{1}
            2D:
                s_{1} = a_{1}*b_{2} - a_{2}*b_{1}

    Args:
        (1, 2) a, b [Vector<float>]: Input vectors for cross product calculation.

    Returns:
        (1) parameter [Vector3<float>]: The resulting vector axb.
    """
    
    if a.size == 2:
        return np.array(a[0]*b[1] - a[1]*b[0], dtype=a.dtype)
    elif a.size == 3:
        return np.array([a[1]*b[2] - a[2]*b[1],
                         a[2]*b[0] - a[0]*b[2],
                         a[0]*b[1] - a[1]*b[0]], dtype=a.dtype)

def Swap(a: float, b: float) -> tp.Tuple[float, float]:
    """
    Description:
        A simple function to swap two numbers.

        Note:
            a, b = b, a

    Args:
        (1, 2) a, b [float]: Input numbers to be swapped.

    Returns:
        (1, 2) parameter [float]: Output swapped numbers.
    """

    return (b, a)

def Binomial_Coefficient(n: int, k: int) -> int:
    """
    Description:
        Calculation binomial coofecient C, from pair of integers n >= k >= 0 and is written (n k). The binomial coefficients are the 
        positive integers that occur as coefficients in the binomial theorem.

            (n k) = n! / (k! * (n - k)!)
            ...

        Simplification of the calculation:
            (n k) = ((n - k + 1) * (n - k + 2) * ... * (n - 1) * (n)) / (1 * 2 * ... * (k - 1) * k)
    
    Args:
        (1) n [int]: Integer number 1 (numerator)
        (2) k [int]: Integer number 2 (denumerator)

    Returns:
        (1) parameter [int]: Binomial coofecient C(n k).
    """
    
    try:
        assert (n >= k)
        
        if k == 0:
            return 1
        elif k == 1:
            return n
        else:
            C_nk = 1
            for i in range(0, k):
                C_nk *= (n - i); C_nk /= (i + 1)

            return C_nk

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] The input condition for the calculation is not satisfied.')
        print(f'[ERROR] The number n ({n}) must be larger than or equal to k ({k}).')

def Factorial(n: int) -> int:
    """
    Description:
        The product of all positive integers less than or equal to a given positive integer.
        The factorial is denoted by {n!}.

        Formula:
            n! = n x (n - 1)!

        For example,
            3! = 3 x 2 x 1 = 6.

        Note:
            The value of 0! is 1.

    Args:
        (1) n [int]: Input number.

    Returns:
        (1) parameter [int]: The factorial of a non-negative integer {n}.
    """
    
    try:
        assert (n >= 0)

        if n == 0:
            return 1
        else:
            out = 1
            for i in range(2, n + 1):
                out *= i

            return out

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] The input condition for the calculation is not satisfied.')
        print('[ERROR] The number n ({n}) must be larger than or equal to 0.')

def Perpendicular_Distance(A: tp.List[float], B: tp.List[float], C: tp.List[float]) -> float:
    """
    Description:
        A function to calculate the shortest distance (or the perpendicular distance). The perpendicular 
        distance is the shortest distance between a point {P} and a line {L}.

        Note:
            If B != C, the perpendicular distance from A to the line through B and C is:
                ||(A - B) x  (C - B)|| / ||C - B||,
            otherwise, 
                ||(A - B)||,
            where x denotes the vector cross product.

    Args:
        (1) A [Vector<float> 1xn]: Point on the line between points B and C.
        (2) B [Vector<float> 1xn]: Start point on the line (P_{0}).
        (3) C [Vector<float> 1xn]: End point on the line (P_{n}).

        Note:
            Where n is the number of dimensions of the point

    Returns:
        (1) parameter [float]: Perpendicular distance.
    """

    if (B == C).all() == True:
        return Euclidean_Norm((A - B))
    else:
        return Euclidean_Norm(Cross((A - B), (C - B)))/Euclidean_Norm((C - B))
    
def Linear_Eqn(a: float, b: float) -> tp.List[float]:
    """
    Description:
        A function to obtain the roots of a linear equation.

        The equation is defined as follows:
            a*x + b = 0.0 -> x = (-b)/(a)
            
    Args:
        (1) a, b [float]: Input values of polynomial coefficients.
                          Note:
                             a: Coefficient of x^1.
                             b: Coefficient of x^0. Constant.

    Returns:
        (1) parameter [Vector<float> 1x1]: The root of the linear equation.
    """

    if a != 0.0:
        return np.array([-b/a], dtype=a.dtype)
    else:
        return np.array([0.0], dtype=a.dtype)

def Quadratic_Eqn(a: float, b: float, c: float) -> tp.List[float]:
    """
    Description:
        A function to obtain the roots of a quadratic equation.

        The equation is defined as follows:
            Solution 1:
                a*x^2 + b*x + c = 0.0 -> x = (-b +- sqrt(b^2 - 4*a*c))/(2*a)

            Solution 2:
                a + b*(1/x) + c*(1/x^2) = 0.0 -> (2*c)/(-b +- sqrt(b^2 - 4*a*c))

                where {b^2 - 4*a*c} is discriminant {D}.

                D > 0: 
                    The roots are real and distinct.
                D = 0: 
                    The roots are real and equal.
                D < 0: 
                    The roots do not exist or the roots are imaginary.
            
    Args:
        (1) a, b, c [float]: Input values of polynomial coefficients.
                             Note:
                                a: Coefficient of x^2.
                                b: Coefficient of x^1.
                                c: Coefficient of x^0. Constant.

    Returns:
        (1) parameter [Vector<float> 1x2]: The roots of the quaratic equation.
    """

    if a != 0.0:
        # Calculate the discriminant {D}.
        D = b*b - 4*a*c
        
        if D >= 0.0:
            # Numerically stable method for solving quadratic equations.
            if b >= 0.0:
                return np.array([(-b - (D)**(0.5)) / (2*a), (2*c)/(-b - (D)**(0.5))], dtype=a.dtype)
            else:
                return np.array([(-b + (D)**(0.5)) / (2*a), (2*c)/(-b + (D)**(0.5))], dtype=a.dtype)
        else:
            # The roots do not exist or the roots are imaginary.
            return np.array([0.0, 0.0], dtype=np.float64)
    else:
        return np.array([0.0, Linear_Eqn(b, c)[0]], dtype=np.float64)

def Inverse_Companion_Matrix(p: tp.List[float]) -> tp.List[tp.List[float]]:
    """
    Description:
        A function to obtain the inverse of the companion matrix {C^(-1)} of polynomial {p} of degree n.

        Formula:
            The companion matrix of the monic polynomial
                p(x) = x^n + a{n-1} * x^(n-1) + ... + a{1} * x + a{0}

            is the square matrix (n, n) defined as
                C(p) = [[0, 0, ..., 0,   -a{0}],
                        [1, 0, ..., 0,   -a{1}],
                        [0, 1, ..., 0,   -a{2}],
                        [., ., .  , .,       .],
                        [., .,  . , .,       .],
                        [., .,   ., .,       .],
                        [0, 0, ..., 1, -a{n-1}]].

            If a{0} is greater then 0, that the inverse of the companion matrix C is defined as
                C^(-1)(p) = [[  -a{1}/a{0}, 1, 0, ..., 0],
                             [  -a{2}/a{0}, 0, 1, ..., 0],
                             [           ., ., ., .  , .],
                             [           ., ., .,  . , .],
                             [           ., ., .,   ., .],
                             [-a{n-1}/a{0}, 0, ., ..., 1],
                             [     -1/a{0}, 0, ., ..., 0]].

            and the characteristic polynomial is
                p(x) = x^n + a{1}/a{0} * x^(n-1) + ... + a{n-1}/a{0} * x + 1/a{0}

        References:
            1. R. A. Horn and C. R. Johnson, "Matrix Analysis".
            2. M. Fiedler, "A note on companion matrices"
            
    Args:
        (1) p [Vector<float> 1xn]: Input values of polynomial coefficients.
                                   Note:
                                    Where {n} is the size of p.

    Returns:
        (1) parameter [Matrix<float> (p.size - 1 x p.size - 1)]: Inverse of the companion matrix {C^(-1)} associated 
                                                                 with the polynomial {p}.
    """

    if isinstance(p, list):
        p = np.array(p, dtype=np.float64)

    # The size {N} of the square matrix.
    N = p.size - 1
    
    # Create the inverse of the companion matrix: C^(-1)
    C = np.zeros((N, N), dtype=p.dtype)
    C[:, 0] = (-1) * (p[1:] / p[0])
    C[0:N-1, 1:] = np.eye(N - 1, dtype=p.dtype)

    return C

def N_Degree_Eqn(p: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to obtain the eigenvalues {lambda} of the inverse of the companion matrix {C^(-1)}, which are defined 
        as the roots of the characteristic polynomial {p}.

        Formula:
            p{0} * x^n + p{1} * x^(n-1) + ... + p{n-1}*x + p{n}
            
    Args:
        (1) p [Vector<float> 1xn]: Input values of polynomial coefficients.
                                   Note:
                                    Where {n} is the size of p.

    Returns:
        (1) parameter [Vector<float> 1xn]: The roots of the polynomial of degree n.
    """

    return np.linalg.eigvals(Inverse_Companion_Matrix(p)).real

def Roots(p: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to obtain the roots of a polynomial of degree n. The function selects the best method 
        to calculate the roots.

            Methods:
                Linear, Quadratic and N-Degree.

    Args:
        (1) p [Vector<float> 1xn]: Input values of polynomial coefficients.
                                   Note:
                                    Where {n} is the size of p.

    Returns:
        (1) parameter [Vector<float> 1xn]: The roots of the polynomial of degree n.
    """

    try:    
        if isinstance(p, list):
            p = np.array(p, dtype=np.float64)
    
        # Check the degree of the input polynomial.
        assert p.size > 1

        if p.size > 2:
            # If the roots cannot be solved using a linear equation.
            i = 0
            for _, (p_i, p_ii) in enumerate(zip(p, p[1:])):
                if p_i == 0 and p_ii != 0:
                    i += 1
                    break

            p_n = p[i:]

            # Selects the calculation method based on the number of polynomial coefficients {p}.
            return {
                2: lambda x: Linear_Eqn(x[0], x[1]),
                3: lambda x: Quadratic_Eqn(x[0], x[1], x[2]),
                'n': lambda x: N_Degree_Eqn(x)
            }[p_n.size if p_n.size < 4 else 'n'](p_n)
        else:
            return Linear_Eqn(p[0], p[1])

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] Insufficient number of input polynomial coefficients.')
