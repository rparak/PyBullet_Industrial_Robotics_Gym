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
File Name: ../RoLE/Kinematics/Core.py
## =========================================================================== ## 
"""

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.: Robotics Library for Everyone (RoLE)
#   ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#   ../RoLE/Kinematics/Utilities/Forward_Kinematics
import RoLE.Kinematics.Utilities.Forward_Kinematics as Utilities
#   ../RoLE/Kinematics/Utilities/General
import RoLE.Kinematics.Utilities.General as General
#   ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Vector3_Cls
#   ../RoLE/Transformation/Utilities/Mathematics
import RoLE.Transformation.Utilities.Mathematics as Mathematics
#   ../RoLE/Transformation/Core
import RoLE.Transformation.Core as Transformation
#   ../RoLE/Interpolation/Utilities
import RoLE.Interpolation.Utilities

"""
Description:
    FORWARD KINEMATICS (FK)
        Forward kinematics kinematics refers to the use of the robot's kinematic equations to calculate 
        the position of the end-effector from specified values of the joint orientations.

    INVERSE KINEMATICS (IK)
        Inverse kinematics deals with the problem of finding the required joint angles to produce a certain desired 
        position and orientation of the end-effector. 

    DH (Denavit-Hartenberg) PARAMETERS: 
        The Denavit - Hartenberg parameters are the four parameters associated with a particular 
        convention for attaching reference frames to the links of a spatial kinematic chain, or robot 
        manipulator.

        | theta | 
        Joint angle (Theta_i). Rotation part in radians.
        
        | a | 
        Link length (a_i). Translation part in meters.

        | d | 
        Joint offset (d_i). Translation part in meters.

        | alpha |  
        Link twist (alpha_i). Rotation part in radians.
"""

def DH_Standard(theta: float, a: float, d: float, alpha: float) -> tp.List[tp.List[float]]:
    """
    Description:
        Standard Denavit-Hartenberg (DH) method. 
        
    Args:
        (1 - 4) theta, a, d, alpha [float]: DH (Denavit-Hartenberg) parameters in the current episode.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix in the current episode.
    """
    
    return HTM_Cls([[np.cos(theta), (-1.0)*(np.sin(theta))*np.cos(alpha),            np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                    [np.sin(theta),          np.cos(theta)*np.cos(alpha), (-1.0)*(np.cos(theta))*(np.sin(alpha)), a*np.sin(theta)],
                    [          0.0,                        np.sin(alpha),                          np.cos(alpha),               d],
                    [          0.0,                                  0.0,                                    0.0,             1.0]], np.float64)

def __Forward_Kinematics_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using the standard Denavit-Hartenberg (DH) method.

        Note:
            DH (Denavit-Hartenberg) table: 
                theta (id: 0), a (id: 1), d (id: 2), alpha (id: 3)

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """

    T_i = Robot_Parameters_Str.T.Base
    for _, (th_i, dh_i, th_i_type, th_i_ax) in enumerate(zip(theta, Robot_Parameters_Str.DH.Standard, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using standard DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Standard(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_i_ax == 'Z':
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

    return T_i @ Robot_Parameters_Str.T.End_Effector

def DH_Modified(theta: float, a: float, d: float, alpha: float) -> tp.List[tp.List[float]]:
    """
    Description:
        Modified Denavit-Hartenberg Method.
        
    Args:
        (1 - 4) theta, a, d, alpha [float]: DH (Denavit-Hartenberg) parameters in the current episode.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix in the current episode.
    """
    
    return HTM_Cls([[np.cos(theta)              ,        (-1.0)*np.sin(theta),                  0.0,                      a],
                    [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), (-1.0)*np.sin(alpha), (-1.0)*np.sin(alpha)*d],
                    [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha),        np.cos(alpha),        np.cos(alpha)*d],
                    [                        0.0,                         0.0,                  0.0,                    1.0]], np.float64)

def __Forward_Kinematics_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using the modified Denavit-Hartenberg (DH) method.
        
        Note:
            DH (Denavit-Hartenberg) table: 
                theta (id: 0), a (id: 1), d (id: 2), alpha (id: 3)

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    T_i = Robot_Parameters_Str.T.Base
    for _, (th_i, dh_i, th_i_type, th_i_ax) in enumerate(zip(theta, Robot_Parameters_Str.DH.Modified, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using modified DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_i_ax == 'Z':
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

    return T_i @ Robot_Parameters_Str.T.End_Effector

def Forward_Kinematics(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                              tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics. The method of calculating depends on the input parameter (2).
        
    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) method [string]: Forward kinematics method (1: Standard, 2: Modified, 3: Fast).
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Vector<bool>]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                      Note:
                                        The value in the vector is "True" if the desired absolute 
                                        joint positions are within the limits, and "False" if they 
                                        are not.
        (2) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """

    # Check that the desired absolute joint positions are not out of limit.
    th_limit_err = General.Check_Theta_Limit(theta, Robot_Parameters_Str)

    # Change of axis direction in individual joints.
    th = (theta * Robot_Parameters_Str.Theta.Direction)

    return {
        'Standard': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Standard(th, r_param_str)),
        'Modified': lambda th, th_err, r_param_str: (th_err, __Forward_Kinematics_Modified(th, r_param_str)),
        'Fast': lambda th, th_err, r_param_str: (th_err, Utilities.FKFast_Solution(th, r_param_str))
    }[method](th, th_limit_err, Robot_Parameters_Str)

def __Get_Individual_Joint_Configuration_Standard(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[tp.List[float]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the standard forward kinematics calculation method.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                               Note:
                                                Where n is the number of joints.
    """
    
    T_i = Robot_Parameters_Str.T.Base; T_cfg = []
    for i, (th_i, dh_i, th_i_type, th_i_ax) in enumerate(zip(theta, Robot_Parameters_Str.DH.Standard, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using standard DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Standard(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_i_ax == 'Z':
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Standard(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

        # Addition of a homogeneous transformation matrix configuration in the current 
        # episode (joint absolute position i).
        if theta.size - 1 == i:
            T_cfg.append(T_i @ Robot_Parameters_Str.T.End_Effector)
        else:
            T_cfg.append(T_i)

    return T_cfg

def __Get_Individual_Joint_Configuration_Modified(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[tp.List[float]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using the modified forward kinematics calculation method.

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        
    Returns:
        (1) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                               Note:
                                                Where n is the number of joints.
    """
    
    T_i = Robot_Parameters_Str.T.Base; T_cfg = []
    for i, (th_i, dh_i, th_i_type, th_i_ax) in enumerate(zip(theta, Robot_Parameters_Str.DH.Modified, Robot_Parameters_Str.Theta.Type, 
                                                             Robot_Parameters_Str.Theta.Axis)):
        # Forward kinematics using modified DH parameters.
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            T_i = T_i @ DH_Modified(dh_i[0] + th_i, dh_i[1], dh_i[2], dh_i[3])
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_i_ax == 'Z':
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1], dh_i[2] - th_i, dh_i[3])
            else:
                # Translation along the X axis.
                T_i = T_i @ DH_Modified(dh_i[0], dh_i[1] + th_i, dh_i[2], dh_i[3])

        # Addition of a homogeneous transformation matrix configuration in the current 
        # episode (joint absolute position i).
        if theta.size - 1 == i:
            T_cfg.append(T_i @ Robot_Parameters_Str.T.End_Effector)
        else:
            T_cfg.append(T_i)

    return T_cfg

def Get_Individual_Joint_Configuration(theta: tp.List[float], method: str, Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                                                              tp.List[tp.List[tp.List[float]]]]:
    """
    Description:
        Get the configuration of the homogeneous transformation matrix of each joint using forward kinematics. The method of calculating 
        the forward kinematics depends on the input parameter (2).

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) method [string]: Forward kinematics method (1: Standard, 2: Modified).
        (3) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Vector<bool>]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                      Note:
                                        The value in the vector is "True" if the desired absolute 
                                        joint positions are within the limits, and "False" if they 
                                        are not.
        (2) parameter [Matrix<float> nx(4x4)]: Configuration homogeneous transformation matrix of each joint.
                                                Note:
                                                    Where n is the number of joints.
    """
    
    # Check that the desired absolute joint positions are not out of limit.
    th_limit_err = General.Check_Theta_Limit(theta, Robot_Parameters_Str)

    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    return {
        'Standard': lambda th, th_err, r_param_str: (th_err, __Get_Individual_Joint_Configuration_Standard(th, r_param_str)),
        'Modified': lambda th, th_err, r_param_str: (th_err, __Get_Individual_Joint_Configuration_Modified(th, r_param_str))
    }[method](th, th_limit_err, Robot_Parameters_Str)

def Get_Geometric_Jacobian(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Get the matrix of the geometric Jacobian (6 x n), where n equals the number of joints.

        The geometric Jacobian (also called the fundamental Jacobian) directly establishes 
        the relationship between joint velocities and the end-effector linear (p_{ee}') and angular (omega_{ee})
        velocities.

        Linear Velocity of the End-Effector:
            p_{ee}' = J_{P}(theta) * theta'
        
        Angular Velocity of the End-Effector:
            omega_{ee} = J_{O}(theta) * theta'

        The Jacobian can be divided into 3x1 columns J_{P} and J_{O} vectors:
            J(theta) = [[J_{P}], = ....
                        [J_{O}]] 

            Revolute Joint = [[z_{i-1} x (p_{ee} - p_{i-1})],
                              [z_{i-1}]]

            Prismatic Joint('Z - Axis') = [[z_{i-1}],
                                           [0.0]]
            Prismatic Joint('X - Axis') = [[(-1) * x_{i-1}],
                                           [0.0]]

    Args:
        (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                        Note:
                                            Where n is the number of joints.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 6xn]: Matrix of the geometric Jacobian (6 x n).
                                            Note: 
                                                Where n is equal to the number of joints.
    """
    
    # Change of axis direction in individual joints.
    th = theta * Robot_Parameters_Str.Theta.Direction

    # Get the configuration of the homogeneous transformation matrix of each joint using the 
    # modified forward kinematics calculation method.
    T_Cfg_Arr = __Get_Individual_Joint_Configuration_Modified(th, Robot_Parameters_Str)

    # Get the translation part from the homogeneous transformation matrix 
    # of the end-effector.
    T_n_p_ee = T_Cfg_Arr[-1].p

    J = np.zeros((6, th.size), dtype=T_n_p_ee.Type); z_i = Vector3_Cls(None, T_n_p_ee.Type)
    for i, (T_Cfg_i, th_i_type, th_i_ax) in enumerate(zip(T_Cfg_Arr, Robot_Parameters_Str.Theta.Type, Robot_Parameters_Str.Theta.Axis)):
        z_i[:] = T_Cfg_i[0:3, 2]
        if th_i_type == 'R':
            # Identification of joint type: R - Revolute
            J_P = z_i.Cross(T_n_p_ee - T_Cfg_i.p)
            J_O = z_i
        elif th_i_type == 'P':
            # Identification of joint type: P - Prismatic
            if th_i_ax == 'Z':
                J_P = z_i
            else:
                # Translation along the X axis.
                J_P = np.float64(-1.0) * Vector3_Cls(T_Cfg_i[0:3, 0], T_n_p_ee.Type)
            J_O = Vector3_Cls([0.0, 0.0, 0.0], z_i.Type)

        # The Jacobian can be divided into 3x1 columns J_{P} and J_{O} vectors:
        J[0:3, i] = J_P.all()
        J[3:6, i] = J_O.all()

    return J

def __IK_N_JT(J: tp.List[tp.List[float]], e_i: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to obtain the absolute joint position (theta) of an individual robotic structure using an inverse kinematics (IK) numerical 
        method called the Jacobian-Transpose (JT).

        Equation:
            theta = alpha * J^T @ e_i,

            where alpha is a appropriate scalar and must be greater than 0. J^T is the transpose of J and e_i is the error (angle axis).

            Expression of the parameter alpha:
                alpha = <e_i, J @ J^T @ e_i> / <J @ J^T @ e_i, J @ J^T @ e_i>.

            Reference:
                Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods, Samuel R. Buss.

        Note:
            To obtain more information about the Args and Returns parameters, please refer to the '__Obtain_Theta_IK_N_Method(..)' function.
    """

    # Auxiliary expression.
    J_T = J.T; x = J @ J_T @ e_i

    # Error avoidance condition.
    if x.any() == False:
        # Because alpha must be greater than 0.0, set alpha
        # as a small number.
        alpha = 1e-5
    else:
        alpha = (e_i @ x) / (x @ x)

    return alpha * J_T @ e_i

def __IK_N_NR(J: tp.List[tp.List[float]], e_i: tp.List[float]) -> tp.List[float]:
    """
    Description:
        A function to obtain the absolute joint position (theta) of an individual robotic structure using an inverse kinematics (IK) numerical 
        method called the Newton-Raphson (NR).

        Equation:
            theta = J^(dagger) @ e_i,

            where J^(dagger) is the pseudoinverse of J, also called the Moore-Penrose inverse of J and e_i is the error (angle axis).

            Reference:
                Modern Robotics: Mechanics, Planning, and Control, Kevin M. Lynch and Frank C. Park

        Note:
            To obtain more information about the Args and Returns parameters, please refer to the '__Obtain_Theta_IK_N_Method(..)' function.
    """

    return np.linalg.pinv(J) @ e_i

def __IK_N_GN(J: tp.List[tp.List[float]], e_i: tp.List[float], W_e: tp.List[tp.List[float]]) -> tp.List[float]:
    """
    Description:
        A function to obtain the absolute joint positions (theta) of an individual robotic structure using an inverse kinematics (IK) numerical 
        method called the Gauss-Newton (GN).

        Equation:
            theta = (H)^(dagger) @ g,

            where H is the Hessian matrix, defined as:
                H = J^T @ W_e @ J

            and W_e is the diagonal weighted matrix and g is the error vector.

            Expression of the parameter g:
                g = J^T @ W_e @ e_i.

            Reference:
                T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method."

        Note:
            To obtain more information about the Args and Returns parameters, please refer to the '__Obtain_Theta_IK_N_Method(..)' function.   
    """
    # Auxiliary expression.
    J_T = J.T; g = J_T @ W_e @ e_i

    return np.linalg.pinv(J_T @ W_e @ J) @ g

def __IK_N_LM(J: tp.List[tp.List[float]], e_i: tp.List[float], W_e: tp.List[tp.List[float]], E: float) -> tp.List[float]:
    """
    Description:
        A function to obtain the absolute joint positions (theta) of an individual robotic structure using an inverse kinematics (IK) numerical 
        method called the Levenberg-Marquardt (LM).

        Equation:
            theta = (H)^(-1) @ g,

            where H is the Hessian matrix, defined as:
                H = J^T @ W_e @ J + W_n,

            and W_e is the diagonal weighted matrix, g is the error vector, and W_n is the damping matrix.

            Note:
                The damping matrix (W_n) ensures that the Hessian matrix (H) is non-singular and positive definite.

            Reference:
                T. Sugihara, "Solvability-Unconcerned Inverse Kinematics by the Levenberg-Marquardt Method."

        Note:
            To obtain more information about the Args and Returns parameters, please refer to the '__Obtain_Theta_IK_N_Method(..)' function.
    """

    # Number of joints.
    n = J.shape[1]

    # Biasing value.
    gamma = 0.0001

    # Auxiliary expression.
    J_T = J.T; g = J_T @ W_e @ e_i

    # Obtain the diagonal damping matrix.
    W_n = E * np.eye(n) + gamma * np.eye(n)

    return np.linalg.inv(J.T @ W_e @ J + W_n) @ g

def __Obtain_Theta_IK_N_Method(method: str, J: tp.List[tp.List[float]], e_i: tp.List[float], W_e: tp.List[tp.List[float]], 
                               E: float) -> tp.List[float]:
    """
    Description:
        A function to obtain the absolute joint positions (theta) of an individual robotic structure using the chosen numerical method.

        Note:
            The pseudoinverse function in numpy is computed using singular value decomposition (SVD), which is robust to 
            singular matrices.

    Args:
        (1) method [string]: Name of the numerical method to be used to calculate the IK solution.
        (2) J [Matrix<float> kxn]: Matrix of the geometric Jacobian (6 x n).
                                    Note: 
                                        Where k is equal to the number of axes and n is equal 
                                        to the number of joints.
        (3) e_i [Vector<float> 1xk]: Vector of an error (angle-axis).
                                        Note: 
                                            Where k is equal to the number of axes.
        (4) W_e [Matrix<float, float> nxn]: Diagonal weight matrix.
                                                Note:
                                                    Where n s equal to the number of joints.
        (5) E [float]: Quadratic (angle-axis) error.

    Returns:
        (1) parameter [Vector<float> 1xn]: Obtained absolute positions of joints in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    return {
        'Jacobian-Transpose': lambda *x: __IK_N_JT(x[0], x[1]),
        'Newton-Raphson': lambda *x: __IK_N_NR(x[0], x[1]),
        'Gauss-Newton': lambda *x: __IK_N_GN(x[0], x[1], x[2]),
        'Levenberg-Marquardt': lambda *x: __IK_N_LM(x[0], x[1], x[2], x[3])
    }[method](J, e_i, W_e, E)

def Inverse_Kinematics_Numerical(TCP_Position: tp.List[tp.List[float]], theta_0: tp.List[float], method: str, 
                                 Robot_Parameters_Str: Parameters.Robot_Parameters_Str, ik_solver_properties: tp.Dict) -> tp.Tuple[tp.Dict, tp.List[float]]:
    """
    Description:
        A function to compute the inverse kinematics (IK) solution of the individual robotic structure using the chosen numerical method.

        Possible numerical methods that can be used include:
            1\ Jacobian-Transpose (JT) Method
            2\ Newton-Raphson (NR) Method
            3\ Gauss-Newton (GN) Method
            4\ Levenberg-Marquardt (LM) Method

        Note:
            The numerical inverse kinematics will be calculated using linear interpolation between the actual and desired positions, defined by the 
            variable 'delta_time.'
            
            If 'delta_time' is equal to 'None,' the calculation will be used without interpolation.

    Args:
        (1) TCP_Position [Matrix<float> 4x4]: The desired TCP (tool center point) in Cartesian coordinates defined 
                                              as a homogeneous transformation matrix.
        (2) theta_0 [Vector<float> 1xn]: Actual absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
        (3) method [string]: Name of the numerical method to be used to calculate the IK solution.
                                Note:
                                    method = 'Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton' or 'Levenberg-Marquardt'
        (4) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
        (5) ik_solver_properties [Dictionary {'delta_time': float or None, 'num_of_iteration': float, 
                                              'tolerance': float}]: The properties of the inverse kinematics solver.
                                                                        Note:
                                                                            'delta_time': The difference (spacing) between 
                                                                                          the time values. If equal to 'None', do not 
                                                                                          use interpolation between the actual and desired 
                                                                                          positions.
                                                                            'num_of_iteration': The number of iterations per 
                                                                                                time instant.
                                                                            'tolerance': The minimum required tolerance per 
                                                                                         time instant.
                                                                                
                                                                            Where time instant is defined by the 'delta_time' variable.

    Returns:
        (1) parameter [Dictionary {'successful': bool, 'iteration': int, 'error': {'position': float, 'orientation': float}, 
                                   'quadratic_error': float, 'is_close_singularity': bool, 'is_self_collision': bool, 
                                   'self_collision_info': Vector<bool> 1xk}]: Information on the best results that were found.
                                                                                Note 1:
                                                                                    Where k is the number of all colliders of the robotic structure.
                                                                                Note 2:
                                                                                    'successful': Information on whether the result was found 
                                                                                                within the required tolerance.
                                                                                    'iteration': Information about the iteration in which the best 
                                                                                                result was found.
                                                                                    'error': Information about the absolute error (position, orientation)
                                                                                    'quadratic_error': Information about Quadratic (angle-axis) error.
                                                                                    'is_close_singularity': Information about whether the Jacobian matrix 
                                                                                                            is close to singularity.
                                                                                    'is_self_collision': Information about whether there are collisions 
                                                                                                        between joints.
                                                                                    'self_collision_info': A vector of information where a collision occurred between 
                                                                                                           the joints of the robotic structure.
        (2) parameter [Vector<float> 1xn]: Obtained the best solution of the absolute positions of the joints in radians / meters.
                                            Note:
                                                Where n is the number of joints. 
    """

    try:
        assert method in ['Jacobian-Transpose', 'Newton-Raphson', 'Gauss-Newton', 'Levenberg-Marquardt']

        if isinstance(TCP_Position, (list, np.ndarray)):
            TCP_Position = Transformation.Homogeneous_Transformation_Matrix_Cls(TCP_Position, np.float64)

        # Diagonal weight matrix.
        #   Note:
        #       Translation(x, y, z) part + Rotation(x, y, z) part.
        W_e = np.diag(np.ones(6))

        # Obtain the actual homogeneous transformation matrix T of the tool center point (TCP).
        T_0 = Forward_Kinematics(theta_0, 'Fast', Robot_Parameters_Str)[1]

        # Express the actual/desired position and orientation of the tool center point (TCP).
        #   Position: p = x, y, z in meters.
        p_0 = T_0.p.all(); p_1 = TCP_Position.p.all()
        #   Orientation: q = w, x, y, z in [-] -> [-1.0, 1.0].
        q_0 = T_0.Get_Rotation('QUATERNION'); q_1 = TCP_Position.Get_Rotation('QUATERNION')

        # If the variable 'delta_time' is not defined, set the variable 't' to 1.0.
        if ik_solver_properties['delta_time'] == None:
            t = np.array([1.0], dtype=np.float64)
        else:
            # Get evenly distributed time values in a given interval.
            #   t_0(0.0) <= t <= t_1(1.0)
            t = np.linspace(0.0, 1.0, int(1.0/ik_solver_properties['delta_time']))
        
        """
        Description:
            Calculate the numerical inverse kinematics using linear interpolation between the actual and desired positions defined by the time 't.'
        """
        iteration = 0.0; th_i = theta_0.copy(); th_i_tmp = theta_0.copy(); T = HTM_Cls(T_0.all(), np.float64)
        for _, t_i in enumerate(t):
            # Obtain the interpolation (Lerp, Slerp) between the given positions and orientations.
            p_i = RoLE.Interpolation.Utilities.Lerp('Explicit', p_0, p_1, t_i)
            q_i = RoLE.Interpolation.Utilities.Slerp('Quaternion', q_0, q_1, t_i)
            
            # Express the homogeneous transformation matrix for the point based on position and rotation.
            T_i = Transformation.Homogeneous_Transformation_Matrix_Cls(None, np.float64).Rotation(q_i.all(), 'QUATERNION').Translation(p_i)

            is_successful = False
            for iteration_i in range(ik_solver_properties['num_of_iteration']):
                # Get the matrix of the geometric Jacobian.
                J = Get_Geometric_Jacobian(th_i, Robot_Parameters_Str)

                # Get an error (angle-axis) vector which represents the translation and rotation.
                e_i = General.Get_Angle_Axis_Error(T_i, T) 

                # Get the quadratic (angle-axis) error which is weighted by the diagonal 
                # matrix W_e.
                E = General.Get_Quadratic_Angle_Axis_Error(e_i, W_e)

                if E < ik_solver_properties['tolerance']:
                    is_successful = True; th_i_tmp = th_i.copy()
                    break
                else: 
                    # Obtain the new theta value using the chosen numerical method.
                    th_i += __Obtain_Theta_IK_N_Method(method, J, e_i, W_e, E)

                # Get the current TCP position of the robotic arm using Forward Kinematics (FK).
                (th_limit_err, T) = Forward_Kinematics(th_i, 'Fast', Robot_Parameters_Str)

                # Check whether the desired absolute joint positions are within the limits.
                for i, th_limit_err_i in enumerate(th_limit_err):
                    if th_limit_err_i == True:
                        th_i[i] = th_i_tmp[i]
                    else:
                        th_i_tmp[i] = th_i[i]

            # Save the number of iterations needed to find the inverse 
            # kinematics (IK) solution at point 'T_i'.
            iteration += iteration_i

            # If the solution was not found within the required tolerance, abort the cycle.
            if is_successful != True:
                break

        # Get the best TCP position of the robotic arm using Forward Kinematics (FK).
        T = Forward_Kinematics(th_i, 'Fast', Robot_Parameters_Str)[1]

        # Check whether the absolute positions of the joints are close to a singularity or if there are collisions 
        # between the joints.
        is_close_singularity = General.Is_Close_Singularity(J)
        (is_self_collision, self_collision_info) = General.Is_Self_Collision(th_i, Robot_Parameters_Str)

        # Obtain the absolute error of position and orientation.
        error = {'position': Mathematics.Euclidean_Norm((TCP_Position.p - T.p).all()), 
                 'orientation': TCP_Position.Get_Rotation('QUATERNION').Distance('Euclidean', T.Get_Rotation('QUATERNION'))}
        
        # Write all the information about the results of the IK solution.
        return ({'successful': is_successful, 'iteration': iteration, 'error': error, 'quadratic_error': E, 
                 'is_close_singularity': is_close_singularity, 'is_self_collision': is_self_collision, 'self_collision_info': self_collision_info}, th_i)

    except AssertionError as error:
        print(f'[ERROR] Information: {error}')
        print('[ERROR] An incorrect name of the method was selected for the numerical inverse kinematics calculation.')
