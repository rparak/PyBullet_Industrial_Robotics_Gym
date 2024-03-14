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
File Name: ../RoLE/Utilities/MOI.py
## =========================================================================== ## 
"""

# Typing (Support for type hints)
import typing as tp

def Cube(m: float, Size: tp.List[float]) -> tp.Tuple[float]:
    """
    Description:
        Obtain the moment of inertia (MOI) for the bounding box (cuboid).

    Args:
        (1) m [float]: The mass of the box.
        (2) Size [Vector<float> 1x3]: The size of the box in three-dimensional space (x, y, z).

    Retruns:
        (1) paramterer [Dictionary{'I_xx': float, 'I_xy': float, etc.}]: The moment of inertia (MOI) of the bounding 
                                                                         box (cuboid).
    """
    
    # Abbreviations.
    division_part = m * 0.0833
    x_square = Size[0] ** 2; y_square = Size[1] ** 2
    z_square = Size[2] ** 2

    return {'I_xx': division_part * (y_square + z_square), 
            'I_xy': 0.0, 
            'I_xz': 0.0,
            'I_yy': division_part * (x_square + z_square), 
            'I_yz': 0.0, 
            'I_zz': division_part * (x_square + y_square)}