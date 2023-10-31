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
File Name: Utilities.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#  ../Lib/Primitives/Core
import Lib.Primitives.Core as Primitives
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

def Get_Min_Max(vertices: tp.List[float]) -> tp.Tuple[tp.List[float], tp.List[float]]:
    """
    Description:
        Get the minimum and maximum X, Y, Z values of the input vertices.

    Args:
        (1) vertices [Vector<float> 8x3]: Vertices of the bounding box (AABB, OBB).

    Returns:
        (1) parameter [Vector<float> 1x3]: Minimum X, Y, Z values of the input vertices.
        (2) parameter [Vector<float> 1x3]: Maximum X, Y, Z values of the input vertices.
    """

    min_vec3 = np.array([vertices[0, 0], vertices[0, 1], vertices[0, 2]], dtype=np.float64)
    max_vec3 = min_vec3.copy()
    
    for _, verts_i in enumerate(vertices[1::]):
        for j, verts_ij in enumerate(verts_i):
            if verts_ij < min_vec3[j]:
                min_vec3[j] = verts_ij

            if verts_ij > max_vec3[j]:
                max_vec3[j] = verts_ij
                
    return (min_vec3, max_vec3)

def CMP(a: float, b: float, epsilon: float) -> bool:
    """
    Description:
        A simple function to compare two numbers.

    Args:
        (1, 2) a, b [float]: Input numbers to be compared.
        (3) epsilon [float]: The machine epsilon. It must be scaled according to the size of the values.
                             Note:
                                epsilon = np.finfo(np.float64).eps or Mathematics.CONST_EPS_32

    Returns:
        (1) parameter [bool]: Output logical variable with certain principles.
    """

    return np.abs(a - b) <= epsilon * np.maximum(1.0, np.maximum(np.abs(a), np.abs(b)))

def Get_Points_of_Intersection(t_min: float, t_max: float, line_segment: Primitives.Line_Segment_Cls) -> tp.List[float]:
    """
    Description:
        Get the points of intersection of the box (AABB, OBB). Using linear interpolation, we can easily 
        calculate where the line segment intersects the box.

        Note:
            The equation of the Lerp function is defined as follows:
                B(t) = (1 - t) * P_{0} + t * P_{1}, where t can take values 0.0 <= t <= 1.0.

    Args:
        (1, 2) t_min, t_max [float]: The largest minimum {t_min} and the smallest maximum {t_max} calculated 
                                     from AABB, OBB raycast function.
        (3) line_segment [Primitives.Line_Segment_Cls]: A specific class for working with a line segment (ray).

    Returns:
        (1) parameter [Vector<float> 2x3]: The points where the line segment intersects the box.
    """

    # Adjust and clamp the values with the defined interval [0.0, 1.0].
    t_0 = Mathematics.Clamp((0.5 - t_min), 0.0, 1.0)
    t_1 = Mathematics.Clamp((0.5 - t_max), 0.0, 1.0)
 
    return np.array([(1 - t_0) * line_segment.a + t_0 * line_segment.b, 
                     (1 - t_1) * line_segment.a + t_1 * line_segment.b], np.float64)

