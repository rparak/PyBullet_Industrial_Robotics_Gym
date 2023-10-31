# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

def Check_Theta_Limit(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[bool]:
    """
    Description:
        Function to check that the desired absolute joint positions are not out of limit.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Vector<bool> 1xn]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                        Note:
                                            The value in the vector is "True" if the desired absolute 
                                            joint positions are within the limits, and "False" if they 
                                            are not.
    """

    th_limit_err = np.zeros(theta.size, dtype=bool)
    for i, (th_i, th_i_limit) in enumerate(zip(theta, Robot_Parameters_Str.Theta.Limit)):
        th_limit_err[i] = False if th_i_limit[0] <= th_i <= th_i_limit[1] else True

    return th_limit_err