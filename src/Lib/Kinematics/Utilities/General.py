# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters
#   ../Lib/Transformation/Core
import Lib.Transformation.Core as Transformation
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core

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

def Get_Angle_Axis_Error(T_desired: tp.List[tp.List[float]], T_current: tp.List[tp.List[float]]) ->tp.List[float]:
    """
    Description:    
        Get an error (angle-axis) vector which represents the translation and rotation from the end-effector's current 
        position (T_current) to the desired position (T_desired).

        The position error is defined as a:
            e_i_p(theta) = d_p_i - p_i(theta),

        where d_p_i is the desired position and p_i is the current position.

        The rotation error is defined as a:

            e_i_R(theta) = alpha(d_R_i * R_i.T(theta)),

        where d_R_i is the desired rotation and R_i is the current rotation. And alpha(R) is angle-axis equivalent of rotation matrix R.

        Reference:
            T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method."

    Args:
        (1) T_desired [Matrix<float> 4x4]: Homogeneous transformation matrix of the desired position/rotation.
        (2) T_current [Matrix<float> 4x4]: Homogeneous transformation matrix of the current position/rotation.

    Returns:
        (1) parameter [Vector<float> 1x6]: Vector of an error (angle-axis) from current to the desired position/rotation.
    """
    
    # Initialization of the output vector, which consists of a translational 
    # and a rotational part.
    e_i = np.zeros(6, dtype=np.float64)

    # 1\ Calculation of position error (e_i_p).
    e_i[:3] = (T_desired.p - T_current.p).all()

    # 2\ Calculation of rotation error (e_i_R).
    R = T_desired.R @ T_current.Transpose().R

    # Trace of an 3x3 square matrix R (tr(R)).
    Tr_R = Transformation.Get_Matrix_Trace(R)
    # Not-zero vector {l}.
    l = Transformation.Vector3_Cls([R[2, 1] - R[1, 2], 
                                    R[0, 2] - R[2, 0], 
                                    R[1, 0] - R[0, 1]], T_desired.Type)
    # Length (norm) of the non-zero vector {l}.
    l_norm = l.Norm()

    if l_norm > Mathematics.CONST_EPS_64:
        e_i[3:] = (np.arctan2(l_norm, Tr_R - 1) * l.all()) / l_norm
    else:
        """
        Condition (Tr_R > 0):
            (r_{11} ,r_{22} ,r_{33}) = (+, +, +), then alpha = Null vector.

            The row e_i[3:] = [0.0, 0.0, 0.0]> is not necessary because 
            <e_i = np.zeros(6, dtype=np.float64)>.
        """
        if Tr_R <= 0:
            e_i[3:] = Mathematics.CONST_MATH_HALF_PI * (Transformation.Get_Matrix_Diagonal(R) + 1)

    return np.array(e_i, dtype=np.float64)

def Get_Quadratic_Angle_Axis_Error(e: tp.List[float], W_e: tp.List[tp.List[float]]) -> float:
    """
    Description:
        Get the quadratic (angle-axis) error which is weighted by the diagonal matrix W_e.

        Reference:
            T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method."
    Args:
        (1) e [Vector<float> 1x6]: Vector of an error (angle-axis) from current to the desired position/rotation.
        (2) W_e [Matrix<float> 6x6]: A diagonal weighting matrix that reflects the priority level of each constraint.

    Returns:
        (1) parameter [float]: Quadratic (angle-axis) error.
    """
    
    return 0.5 * e @ W_e @ e

def Is_Self_Collision(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[bool]:
    """
    Description:
        A function to obtain information whether there is a collision between the joints of the robotic structure.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Vector<bool> 1xk]: A vector of errors where a collision occurred between the joints of the robotic structure.
                                            Note:
                                                Where k is the number of all colliders of the robotic structure.
    """

    # Get a list of base and joint colliders.
    Base_Collider = list(Robot_Parameters_Str.Collider.Base.values()); Theta_Collider = list(Robot_Parameters_Str.Collider.Theta.values())
    
    # Transformation of the base collider according to the input homogeneous transformation matrix.
    Base_Collider[0].Transformation(Robot_Parameters_Str.T.Base)
 
    # Obtain the individual (theta) configuration of the homogeneous matrix of each joint using forward kinematics
    T_Arr = Lib.Kinematics.Core.Get_Individual_Joint_Configuration(theta, 'Modified', Robot_Parameters_Str)[1]

    # Transformation of the joint colliders according to the input homogeneous transformation matrix.
    for _, (T_i, th_collider_i) in enumerate(zip(T_Arr, Theta_Collider)):
        th_collider_i.Transformation(T_i)

    # Concatenate all colliders (base, joint) into single array according to a predefined constraint.
    if Robot_Parameters_Str.External_Axis == True:
        Base_Collider[1].Transformation(T_Arr[0])
        All_Colliders = np.concatenate(([Base_Collider[0], Theta_Collider[0], Base_Collider[1]], 
                                         Theta_Collider[1::]))
    else:
        All_Colliders = np.concatenate((Base_Collider, Theta_Collider))

    # Check whether the 3D primitives (bounding boxes AABB, OBB) overlap or not.
    is_collision = np.zeros(All_Colliders.size, dtype=bool)
    for i, Collider_i in enumerate(All_Colliders):
        for j, Collider_j in enumerate(All_Colliders[(i + 1) + Robot_Parameters_Str.Collider.Offset::], 
                                       start=(i + 1) + Robot_Parameters_Str.Collider.Offset):
            if Collider_i.Overlap(Collider_j) == True:
                # Set the individual parts where the collision occurs.
                is_collision[i] = True; is_collision[j] = True

    return is_collision