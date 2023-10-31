# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot as Parameters

def __FKF_Universal_Robots_UR3(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the Universal Robots UR3 robotic arm.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Cosine / Sine functions: Default notation.
    c_th_0 = np.cos(theta[0]); c_th_1 = np.cos(theta[1]); c_th_3 = np.cos(theta[3]); c_th_4 = np.cos(theta[4]); c_th_5 = np.cos(theta[5])
    s_th_0 = np.sin(theta[0]); s_th_1 = np.sin(theta[1]); s_th_3 = np.sin(theta[3]); s_th_4 = np.sin(theta[4]); s_th_5 = np.sin(theta[5])
    # Angles:
    th_12_t1 = theta[1] + theta[2]; th_13_t1 = th_12_t1 + theta[3]; th_14_t1 = th_13_t1 - theta[4]; th_14_t2 = th_13_t1 + theta[4]
    th_04_t1 = -theta[0] + th_14_t1; th_04_t2 = -theta[0] + th_14_t2; th_04_t3 = theta[0] + th_14_t1
    th_04_t4 = theta[0] + th_14_t2; th_012_t1 = theta[0] + th_12_t1; th_012_t2 = -theta[0] + th_12_t1
    # Additional Sine function:
    s_th_04 = s_th_0*s_th_4; s_th_13_t1 = np.sin(th_13_t1); s_th_04_t1 = np.sin(th_04_t1); s_th_012_t1 = np.sin(th_012_t2)
    s_th_04_t2 = np.sin(th_04_t3); s_th_04_t3 = np.sin(th_04_t4)
    # Additional Cosine function:
    c_th_04_t1 = 0.25*np.cos(th_04_t1); c_th_04_t2 = 0.25*np.cos(th_04_t2); c_th_04_t3 = 0.25*np.cos(th_04_t3); c_th_04_t4 = 0.25*np.cos(th_04_t4)
    c_th_012_t1 = np.cos(th_012_t1); c_th_012_t2 = 0.5*c_th_012_t1; c_th_012_t3 = np.cos(th_012_t2); c_th_012_t4 = 0.5*c_th_012_t3
    c_th_13_t1 = np.cos(th_13_t1); c_th_04_t5 = c_th_0*c_th_4

    # Mutual abbreviations:
    #   Duplicates P0
    s_th_0_c_th_4 = s_th_0*c_th_4; s_th_4_c_th_0 = s_th_4*c_th_0; s_th413_t1 = s_th_4*s_th_13_t1; s_th513_t1 = s_th_5*s_th_13_t1
    #   Duplicates P1
    aux_id_0 = (s_th_04 + c_th_04_t1 + c_th_04_t2 + c_th_04_t3 + c_th_04_t4)

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = aux_id_0*c_th_5 - s_th513_t1*c_th_0
    T[0,1] = -aux_id_0*s_th_5 - s_th_13_t1*c_th_0*c_th_5
    T[0,2] = s_th_0_c_th_4 + 0.25*s_th_04_t1 - 0.25*np.sin(th_04_t2) + 0.25*s_th_04_t2 - 0.25*s_th_04_t3
    T[0,3] = 0.0853499957780968*(0.5*s_th_012_t1 + 0.5*np.sin(th_012_t1))*c_th_3 + 0.0853499957780968*(c_th_012_t4 + c_th_012_t2)*s_th_3 + 0.0819000010962388*s_th_0_c_th_4 + 0.112349998365105*s_th_0 + 0.0204750019763515*s_th_04_t1 - 0.020475*np.sin(th_04_t2) + 0.020475*s_th_04_t2 - 0.0204749983084106*s_th_04_t3 - 0.24365000451957*c_th_0*c_th_1 - 0.106625002810062*c_th_012_t3 - 0.106624993595327*c_th_012_t1
    T[1,0] = (s_th_0_c_th_4*c_th_13_t1 - s_th_4_c_th_0)*c_th_5 - s_th_0*s_th513_t1
    T[1,1] = (-s_th_0_c_th_4*c_th_13_t1 + s_th_4_c_th_0)*s_th_5 - s_th_0*s_th_13_t1*c_th_5
    T[1,2] = -s_th_04*c_th_13_t1 - c_th_04_t5
    T[1,3] = 0.0853499957780968*(c_th_012_t4 - c_th_012_t2)*c_th_3 + 0.0853499957780968*s_th_0*s_th_3*np.cos(th_12_t1) - 0.0819000010962388*s_th_0*s_th_4*c_th_13_t1 - 0.24365000451957*s_th_0*c_th_1 + 0.106625*s_th_012_t1 - 0.106624995858687*np.sin(th_012_t1) - 0.0819000010962388*c_th_04_t5 - 0.112349998365105*c_th_0
    T[2,0] = s_th_5*c_th_13_t1 + s_th_13_t1*c_th_4*c_th_5
    T[2,1] = -s_th513_t1*c_th_4 + c_th_5*c_th_13_t1
    T[2,2] = -s_th413_t1
    T[2,3] = -0.24365000451957*s_th_1 - 0.0819000010962388*s_th413_t1 - 0.213249996297229*np.sin(th_12_t1) - 0.0853499957780968*c_th_13_t1 + 0.151899988439879
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def __FKF_ABB_IRB_120(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the ABB IRB 120 robotic arm.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """

    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Cosine / Sine functions: Default notation.
    c_th_0 = np.cos(theta[0]); c_th_2 = np.cos(theta[2]); c_th_3 = np.cos(theta[3]); c_th_4 = np.cos(theta[4])
    s_th_0 = np.sin(theta[0]); s_th_2 = np.sin(theta[2]); s_th_3 = np.sin(theta[3]); s_th_4 = np.sin(theta[4])
    # Angles:
    th_12_t1 = theta[1] + theta[2]; th_12_t2 = -theta[1] - theta[2]; th_0123_t1 = theta[0] + th_12_t2 + theta[3]
    th_0123_t2 = theta[0] + th_12_t1 + theta[3]; th_0123_t3 = -theta[0] + th_12_t1 + theta[3]; th_0123_t4 = theta[0] + th_12_t1 - theta[3]
    # Additional Sine functions:
    s_th_1PI2 = np.sin(theta[1] - 1.5707963267948966); s_th_5PI = np.sin(theta[5] + 3.141592653589793); s_th_12 = np.sin(th_12_t1)
    s_th_0123_t1 = np.sin(th_0123_t3); s_th_0123_t2 = np.sin(th_0123_t1)
    s_th_0123_t3 = np.sin(th_0123_t4); s_th_0123_t4 = np.sin(th_0123_t2)
    # Additional Cosine functions:
    c_th_1PI2 = np.cos(theta[1] - 1.5707963267948966); c_th_5PI = np.cos(theta[5] + 3.141592653589793); c_th_12_t1 = np.cos(th_12_t1)
    c_th_0123_t1 = np.cos(th_0123_t3); c_th_0123_t2 = np.cos(th_0123_t1)
    c_th_0123_t3 = np.cos(th_0123_t4); c_th_0123_t4 = np.cos(th_0123_t2)

    # Mutual abbreviations:
    #   Duplicates P0
    s_th_03 = s_th_0*s_th_3; v_s_th_0123_t1 = 0.25*s_th_0123_t1; v_s_th_0123_t2 = 0.25*s_th_0123_t2
    v_s_th_0123_t3 = 0.25*s_th_0123_t3; v_s_th_0123_t4 = 0.25*s_th_0123_t4
    aux_id_0 = (s_th_03 + v_s_th_0123_t1 - v_s_th_0123_t2 + v_s_th_0123_t3 + v_s_th_0123_t4)
    #   Duplicates P1
    s_th_4_c_th_3 = s_th_0*c_th_3; v_c_th_0123_t1 = 0.25*c_th_0123_t1
    v_c_th_0123_t2 = 0.25*c_th_0123_t2; v_c_th_0123_t3 = 0.25*c_th_0123_t3; v_c_th_0123_t4 = 0.25*c_th_0123_t4
    aux_id_1 = (s_th_4_c_th_3 + v_c_th_0123_t1 - v_c_th_0123_t2 - v_c_th_0123_t3 + v_c_th_0123_t4)
    #   Duplicates P2
    s_th_4_c_th_012 = s_th_4*c_th_0*c_th_12_t1; c_th_412 = c_th_4*c_th_12_t1; v_s_th_2_t1 = 0.07*s_th_2; s_th_1PI2_c_th_0 = s_th_1PI2*c_th_0
    v_s_th_2_t2 = 0.301999979970156*s_th_2; c_th_01PI2 = c_th_0*c_th_1PI2; c_th_21PI2 = c_th_2*c_th_1PI2; s_th_3_c_th_0 = s_th_3*c_th_0
    s_th_02 = s_th_0*s_th_2; s_th_1PI2_c_th_2 = s_th_1PI2*c_th_2; s_th_412 = s_th_4*s_th_12
    s_th_4_c_th_312 = s_th_4*c_th_3*c_th_12_t1
    #   Duplicates P3
    aux_id_2 = aux_id_0*s_th_4; aux_id_3 = c_th_3*c_th_412; aux_id_4 = (s_th_3_c_th_0 - v_c_th_0123_t1 - v_c_th_0123_t2 + v_c_th_0123_t3 + v_c_th_0123_t4); 
    aux_id_5 = (-aux_id_4*c_th_4 + s_th_0*s_th_4*c_th_12_t1); aux_id_6 = (v_s_th_0123_t1 + v_s_th_0123_t2 + v_s_th_0123_t3 - v_s_th_0123_t4 + c_th_0*c_th_3)

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = (aux_id_0*c_th_4 + s_th_4_c_th_012)*c_th_5PI + aux_id_1*s_th_5PI
    T[0,1] = ((-s_th_03 - v_s_th_0123_t1 + v_s_th_0123_t2 - v_s_th_0123_t3 - v_s_th_0123_t4)*c_th_4 - s_th_4_c_th_012)*s_th_5PI + aux_id_1*c_th_5PI
    T[0,2] = -aux_id_2 + c_th_0*c_th_412
    T[0,3] = -0.072*aux_id_2 - v_s_th_2_t1*s_th_1PI2_c_th_0 - v_s_th_2_t2*c_th_01PI2 - 0.301999979970156*s_th_1PI2_c_th_0*c_th_2 + 0.07*c_th_0*c_th_21PI2 + 0.072*c_th_0*c_th_412 + 0.27*c_th_01PI2
    T[1,0] = aux_id_5*c_th_5PI - aux_id_6*s_th_5PI
    T[1,1] = -aux_id_5*s_th_5PI - aux_id_6*c_th_5PI
    T[1,2] = aux_id_4*s_th_4 + s_th_0*c_th_412
    T[1,3] = 0.072*aux_id_4*s_th_4 - 0.07*s_th_02*s_th_1PI2 - 0.301999979970156*s_th_02*c_th_1PI2 - 0.301999979970156*s_th_0*s_th_1PI2_c_th_2 + 0.07*s_th_0*c_th_21PI2 + 0.072*s_th_0*c_th_412 + 0.27*s_th_0*c_th_1PI2
    T[2,0] = (-s_th_412 + aux_id_3)*c_th_5PI - s_th_3*s_th_5PI*c_th_12_t1
    T[2,1] = (s_th_412 - aux_id_3)*s_th_5PI - s_th_3*c_th_12_t1*c_th_5PI
    T[2,2] = -s_th_4_c_th_312 - s_th_12*c_th_4
    T[2,3] = v_s_th_2_t2*s_th_1PI2 - v_s_th_2_t1*c_th_1PI2 - 0.072*s_th_4_c_th_312 - 0.07*s_th_1PI2_c_th_2 - 0.27*s_th_1PI2 - 0.072*s_th_12*c_th_4 - 0.301999979970156*c_th_21PI2 + 0.29
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def __FKF_ABB_IRB_120_L_Ax(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the ABB IRB 120 robotic arm 
        supplemented with a 7th linear axis.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Cosine / Sine functions: Default notation.
    c_th_1 = np.cos(theta[1]); c_th_3 = np.cos(theta[3]); c_th_4 = np.cos(theta[4]); c_th_5 = np.cos(theta[5])
    s_th_1 = np.sin(theta[1]); s_th_3 = np.sin(theta[3]); s_th_4 = np.sin(theta[4]); s_th_5 = np.sin(theta[5])
    # Angles:
    th_23_t1 = theta[2] + theta[3]
    th_1234_t11 = theta[1] + th_23_t1 + theta[4]; th_1234_t12 = -theta[1] + th_23_t1 + theta[4]
    th_1234_t2 = theta[1] - theta[2] - theta[3] + theta[4]
    th_1234_t3 = theta[1] + th_23_t1 - theta[4]
    # Additional Sine functions:
    s_th_1234_t12 = np.sin(th_1234_t12); s_th_1234_t2 = np.sin(th_1234_t2); s_th_1234_t3 = np.sin(th_1234_t3)
    s_th_1234_t11 = np.sin(th_1234_t11); s_th_23_t1 = np.sin(th_23_t1); s_th_3_v = 0.301999979970156*s_th_3
    s_th_1_v = 0.301999979970156*s_th_1
    s_th_6PI = np.sin(theta[6] + 3.141592653589793); s_th_2PI2 = np.sin(theta[2] - 1.5707963267948966)
    # Additional Cosine functions:
    c_th_23_t1 = np.cos(th_23_t1); c_th_cos_1234_t12 = np.cos(th_1234_t12); c_th_1234_t2 = np.cos(th_1234_t2)
    c_th_1234_t3 = np.cos(th_1234_t3); c_th_cos_1234_t11 = np.cos(th_1234_t11); c_th_123_t1 = c_th_1*c_th_23_t1
    c_th_123_t2 = c_th_5*c_th_23_t1; c_th_123_t3 = c_th_1*c_th_123_t2
    c_th_6PI = np.cos(theta[6] + 3.141592653589793); c_th_2PI2 = np.cos(theta[2] - 1.5707963267948966)
    c_th_32PI = c_th_3*c_th_2PI2

    # Mutual abbreviations:
    #   Duplicates P0
    s_th_14 = s_th_1*s_th_4
    v_s_th_1234_t1 = 0.25*s_th_1234_t12; v_s_th_1234_t2 = 0.25*s_th_1234_t2
    v_s_th_1234_t3 = 0.25*s_th_1234_t3; v_s_th_1234_t4 = 0.25*s_th_1234_t11
    #   Duplicates P1
    v_c_th_1234_t1 = 0.25*c_th_cos_1234_t12; v_c_th_1234_t2 = 0.25*c_th_1234_t2
    v_c_th_1234_t3 = 0.25*c_th_1234_t3; v_c_th_1234_t4 = 0.25*c_th_cos_1234_t11
    #   Duplicates P2
    aux_id_1 = (s_th_14 + v_s_th_1234_t1 - v_s_th_1234_t2 + v_s_th_1234_t3 + v_s_th_1234_t4)
    aux_id_2 = (s_th_1*c_th_4 + v_c_th_1234_t1 - v_c_th_1234_t2 - v_c_th_1234_t3 + v_c_th_1234_t4)
    aux_id_3 = s_th_4*c_th_1 - v_c_th_1234_t1 - v_c_th_1234_t2 + v_c_th_1234_t3 + v_c_th_1234_t4
    aux_id_4 = (aux_id_3)*s_th_5; aux_id_5 = s_th_1*s_th_5*c_th_23_t1
    aux_id_6 = v_s_th_1234_t1 + v_s_th_1234_t2 + v_s_th_1234_t3 - v_s_th_1234_t4 + c_th_1*c_th_4

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = (aux_id_1*c_th_5 + s_th_5*c_th_123_t1)*c_th_6PI + aux_id_2*s_th_6PI
    T[0,1] = ((-s_th_14 - v_s_th_1234_t1 + v_s_th_1234_t2 - v_s_th_1234_t3 - v_s_th_1234_t4)*c_th_5 - s_th_5*c_th_123_t1)*s_th_6PI + aux_id_2*c_th_6PI
    T[0,2] = -aux_id_1*s_th_5 + c_th_123_t3
    T[0,3] = theta[0] - 0.072*aux_id_1*s_th_5 - 0.07*s_th_3*s_th_2PI2*c_th_1 - s_th_3_v*c_th_1*c_th_2PI2 - 0.301999979970156*s_th_2PI2*c_th_1*c_th_3 + 0.07*c_th_1*c_th_32PI + 0.072*c_th_123_t3 + 0.27*c_th_1*c_th_2PI2
    T[1,0] = (-(aux_id_3)*c_th_5 + aux_id_5)*c_th_6PI - (aux_id_6)*s_th_6PI
    T[1,1] = -(-(aux_id_3)*c_th_5 + aux_id_5)*s_th_6PI - (aux_id_6)*c_th_6PI
    T[1,2] = aux_id_4 + s_th_1*c_th_123_t2
    T[1,3] = 0.072*aux_id_4 - 0.07*s_th_1*s_th_3*s_th_2PI2 - s_th_1_v*s_th_3*c_th_2PI2 - s_th_1_v*s_th_2PI2*c_th_3 + 0.07*s_th_1*c_th_32PI + 0.072*s_th_1*c_th_123_t2 + 0.27*s_th_1*c_th_2PI2
    T[2,0] = (-s_th_5*s_th_23_t1 + c_th_4*c_th_123_t2)*c_th_6PI - s_th_4*s_th_6PI*c_th_23_t1
    T[2,1] = -(-s_th_5*s_th_23_t1 + c_th_4*c_th_123_t2)*s_th_6PI - s_th_4*c_th_23_t1*c_th_6PI
    T[2,2] = -s_th_5*c_th_4*c_th_23_t1 - s_th_23_t1*c_th_5
    T[2,3] = s_th_3_v*s_th_2PI2 - 0.07*s_th_3*c_th_2PI2 - 0.072*s_th_5*c_th_4*c_th_23_t1 - 0.07*s_th_2PI2*c_th_3 - 0.27*s_th_2PI2 - 0.072*s_th_23_t1*c_th_5 - 0.301999979970156*c_th_32PI + 0.402999989697838
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def __FKF_ABB_IRB_14000_R(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the ABB IRB 1400 (YuMi) robotic arm.

        Note:
            YuMi's right hand.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
        

    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Cosine / Sine functions: Default notation.
    c_th_0 = np.cos(theta[0]); c_th_1 = np.cos(theta[1]); c_th_2 = np.cos(theta[2]); c_th_5 = np.cos(theta[5]); c_th_6 = np.cos(theta[6])
    s_th_0 = np.sin(theta[0]); s_th_1 = np.sin(theta[1]); s_th_2 = np.sin(theta[2]); s_th_5 = np.sin(theta[5]); s_th_6 = np.sin(theta[6])
    # Angles:
    th_02_t1 = theta[0] - theta[1] + theta[2]; th_02_t2 = -theta[0] + theta[1] + theta[2]; th_02_t3 = theta[0] + theta[1] + theta[2]
    th_02_t4 = theta[0] + theta[1] - theta[2]
    # Additional Sine function:
    s_th_3PI2 = np.sin(theta[3] - 1.5707963267948966); s_th_4PI = np.sin(theta[4] + 3.141592653589793)
    s_th_02_t1 = np.sin(th_02_t1); s_th_02_t2 = np.sin(th_02_t2); s_th_02_t3 = np.sin(th_02_t3)
    s_th_02_t4 = np.sin(th_02_t4); s_th_01_t1 = s_th_0*s_th_1
    # Additional Cosine function:
    c_th_3PI2 = np.cos(theta[3] - 1.5707963267948966); c_th_4PI = np.cos(theta[4] + 3.141592653589793)
    c_th_02_t1 = np.cos(th_02_t1); c_th_02_t2 = np.cos(th_02_t2); c_th_02_t3 = np.cos(th_02_t3)
    c_th_02_t4 = np.cos(th_02_t4)

    # Mutual abbreviations:
    #   Duplicates P0
    v_s_th_02_t1 = 0.25*s_th_02_t1; v_s_th_02_t2 = 0.25*s_th_02_t2; v_s_th_02_t3 = 0.25*s_th_02_t3
    v_s_th_02_t4 = 0.25*s_th_02_t4
    v_c_th_02_t1 = 0.25*c_th_02_t1; v_c_th_02_t2 = 0.25*c_th_02_t2; v_c_th_02_t3 = 0.25*c_th_02_t3
    v_c_th_02_t4 = 0.25*c_th_02_t4
    #   Duplicates P1
    s_th_02_t11 = -s_th_0*s_th_2; s_th_02_t12 = s_th_0*s_th_2; s_th_124PI = s_th_1*s_th_2*s_th_4PI; s_th_0_c_th_2 = s_th_0*c_th_2
    #   Duplicates P2
    aux_id_0 = s_th_1*c_th_2*c_th_3PI2; aux_id_1 = s_th_3PI2*c_th_1; aux_id_2 = aux_id_0 + aux_id_1
    aux_id_3 = s_th_1*s_th_3PI2*c_th_0; aux_id_4 = (s_th_02_t11 + v_c_th_02_t2 + v_c_th_02_t1 + v_c_th_02_t4 + v_c_th_02_t3)
    aux_id_33 = (s_th_0_c_th_2 + v_s_th_02_t2 + v_s_th_02_t1 - v_s_th_02_t4 + v_s_th_02_t3)
    aux_id_5 = aux_id_33*s_th_4PI; aux_id_6 = aux_id_4*c_th_3PI2; aux_id_7 = (-aux_id_4*s_th_3PI2 - s_th_1*c_th_0*c_th_3PI2)*s_th_5; aux_id_8 = aux_id_6 - aux_id_3
    aux_id_9 = aux_id_33*c_th_4PI; aux_id_10 = (s_th_02_t12 - v_c_th_02_t2 - v_c_th_02_t1 - v_c_th_02_t4 - v_c_th_02_t3); aux_id_11 = (aux_id_10*c_th_3PI2 + aux_id_3)
    aux_id_12 = (aux_id_11*c_th_4PI - aux_id_5); aux_id_13 = aux_id_10*s_th_3PI2 - s_th_1*c_th_0*c_th_3PI2; aux_id_14 = (aux_id_13)*c_th_5
    aux_id_15 = (s_th_2*c_th_0 - v_s_th_02_t2 + v_s_th_02_t1 + v_s_th_02_t4 + v_s_th_02_t3)*c_th_3PI2; aux_id_23 = s_th_01_t1*s_th_3PI2
    aux_id_16 = (s_th_2*c_th_0 - v_s_th_02_t2 + v_s_th_02_t1 + v_s_th_02_t4 + v_s_th_02_t3)*s_th_3PI2; aux_id_17 = (-aux_id_15 + aux_id_23)*c_th_4PI
    aux_id_18 = (-c_th_0*c_th_2 + v_c_th_02_t2 - v_c_th_02_t1 + v_c_th_02_t4 - v_c_th_02_t3)*s_th_4PI; aux_id_19 = (aux_id_16 + s_th_01_t1*c_th_3PI2)
    aux_id_20 = aux_id_19*s_th_5; aux_id_21 = -aux_id_17 + aux_id_18; aux_id_22 = (aux_id_21)*s_th_5; aux_id_24 = (aux_id_2)*c_th_4PI; aux_id_25 = -aux_id_24 - s_th_124PI
    aux_id_26 = (aux_id_25)*c_th_5; aux_id_27 = (aux_id_25)*s_th_5; aux_id_28 = -s_th_1*s_th_3PI2*c_th_2 + c_th_1*c_th_3PI2; aux_id_29 = (aux_id_28)*s_th_5; aux_id_30 = (aux_id_28)*c_th_5
    aux_id_31 = (aux_id_2)*s_th_4PI - s_th_1*s_th_2*c_th_4PI; aux_id_32 = aux_id_10*c_th_3PI2 + aux_id_3; aux_id_34 = aux_id_32*c_th_4PI; aux_id_35 = (-aux_id_34 + aux_id_5)
    aux_id_36 = (c_th_0*c_th_2 - v_c_th_02_t2 + v_c_th_02_t1 - v_c_th_02_t4 + v_c_th_02_t3); aux_id_37 = s_th_0*s_th_1*s_th_3PI2; aux_id_38 = -(aux_id_0 + aux_id_1)*c_th_4PI
    aux_id_39 = s_th_0*s_th_1*c_th_3PI2

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = -((-(aux_id_8)*c_th_4PI - aux_id_5)*c_th_5 + aux_id_7)*c_th_6 - ((aux_id_8)*s_th_4PI - aux_id_9)*s_th_6
    T[0,1] = (((-aux_id_6 + aux_id_3)*c_th_4PI - aux_id_5)*c_th_5 + aux_id_7)*s_th_6 - ((aux_id_4*c_th_3PI2 - aux_id_3)*s_th_4PI - aux_id_9)*c_th_6
    T[0,2] = -aux_id_12*s_th_5 + aux_id_14
    T[0,3] = 0.036*aux_id_35*s_th_5 - 0.027*aux_id_35*c_th_5 + 0.027*(aux_id_13)*s_th_5 + 0.036*(aux_id_13)*c_th_5 - 0.027*aux_id_34 + 0.265*aux_id_10*s_th_3PI2 - 0.0405*aux_id_10*c_th_3PI2 + 0.027*aux_id_5 - 0.0405*s_th_02_t12 - 0.0405*aux_id_3 - 0.265*s_th_1*c_th_0*c_th_3PI2 + 0.251500010453034*s_th_1*c_th_0 + 0.03*c_th_0*c_th_1 - 0.03*c_th_0 + 0.010125*c_th_02_t2 + 0.0101250012797019*c_th_02_t1 + 0.010125*c_th_02_t4 + 0.0101249986747384*c_th_02_t3
    T[1,0] = (-(aux_id_17 + aux_id_36*s_th_4PI)*c_th_5 + aux_id_20)*c_th_6 + ((-aux_id_15 + aux_id_23)*s_th_4PI - aux_id_36*c_th_4PI)*s_th_6
    T[1,1] = (-(aux_id_21)*c_th_5 - aux_id_20)*s_th_6 + ((-aux_id_15 + aux_id_23)*s_th_4PI + (-c_th_0*c_th_2 + v_c_th_02_t2 - v_c_th_02_t1 + v_c_th_02_t4 - v_c_th_02_t3)*c_th_4PI)*c_th_6
    T[1,2] = aux_id_22 - aux_id_19*c_th_5
    T[1,3] = 0.036*(-(-aux_id_15 + aux_id_37)*c_th_4PI + aux_id_18)*s_th_5 - 0.027*(-(-aux_id_15 + aux_id_37)*c_th_4PI + aux_id_18)*c_th_5 - 0.027*(aux_id_16 + aux_id_39)*s_th_5 - 0.036*(aux_id_16 + aux_id_39)*c_th_5 + 0.027*(aux_id_15 - aux_id_37)*c_th_4PI - 0.265*aux_id_16 + 0.0405*aux_id_15 - 0.027*aux_id_36*s_th_4PI - 0.0405*aux_id_37 - 0.265*aux_id_39 + 0.251500010453034*s_th_0*s_th_1 + 0.03*s_th_0*c_th_1 - 0.03*s_th_0 + 0.0405*s_th_2*c_th_0 - 0.010125*s_th_02_t2 + 0.010125*s_th_02_t1 + 0.010125*s_th_02_t4 + 0.010125*s_th_02_t3
    T[2,0] = (aux_id_26 + aux_id_29)*c_th_6 + (aux_id_31)*s_th_6
    T[2,1] = -(aux_id_26 + aux_id_29)*s_th_6 + (aux_id_31)*c_th_6
    T[2,2] = aux_id_27 - aux_id_30
    T[2,3] = 0.036*(aux_id_38- s_th_124PI)*s_th_5 - 0.027*(aux_id_38- s_th_124PI)*c_th_5 - 0.027*(aux_id_28)*s_th_5 - 0.036*(aux_id_28)*c_th_5 - 0.027*(aux_id_0 + aux_id_1)*c_th_4PI - 0.027*s_th_124PI + 0.265*s_th_1*s_th_3PI2*c_th_2 - 0.0405*aux_id_0 - 0.0405*s_th_1*c_th_2 - 0.03*s_th_1 - 0.0405*aux_id_1 - 0.265*c_th_1*c_th_3PI2 + 0.251500010453034*c_th_1 + 0.1
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def __FKF_ABB_IRB_14000_L(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the ABB IRB 1400 (YuMi) robotic arm.

        Note:
            YuMi's left hand.

    Args:th_02_t1
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Cosine / Sine functions: Default notation.
    c_th_0 = np.cos(theta[0]); c_th_1 = np.cos(theta[1]); c_th_2 = np.cos(theta[2]); c_th_5 = np.cos(theta[5]); c_th_6 = np.cos(theta[6])
    s_th_0 = np.sin(theta[0]); s_th_1 = np.sin(theta[1]); s_th_2 = np.sin(theta[2]); s_th_5 = np.sin(theta[5]); s_th_6 = np.sin(theta[6])
    # Angles:
    th_02_t1 = theta[0] - theta[1] + theta[2]; th_02_t2 = -theta[0] + theta[1] + theta[2]; th_02_t3 = theta[0] + theta[1] + theta[2]
    th_02_t4 = theta[0] + theta[1] - theta[2]
    # Additional Sine function:
    s_th_3PI2 = np.sin(theta[3] - 1.5707963267948966); s_th_4PI = np.sin(theta[4] + 3.141592653589793)
    s_th_02_t1 = np.sin(th_02_t1); s_th_02_t2 = np.sin(th_02_t2); s_th_02_t3 = np.sin(th_02_t3)
    s_th_02_t4 = np.sin(th_02_t4); s_th_01_t1 = s_th_0*s_th_1
    # Additional Cosine function:
    c_th_3PI2 = np.cos(theta[3] - 1.5707963267948966); c_th_4PI = np.cos(theta[4] + 3.141592653589793)
    c_th_02_t1 = np.cos(th_02_t1); c_th_02_t2 = np.cos(th_02_t2); c_th_02_t3 = np.cos(th_02_t3)
    c_th_02_t4 = np.cos(th_02_t4)

    # Mutual abbreviations:
    #   Duplicates P0
    v_s_th_02_t1 = 0.25*s_th_02_t1; v_s_th_02_t2 = 0.25*s_th_02_t2; v_s_th_02_t3 = 0.25*s_th_02_t3
    v_s_th_02_t4 = 0.25*s_th_02_t4
    v_c_th_02_t1 = 0.25*c_th_02_t1; v_c_th_02_t2 = 0.25*c_th_02_t2; v_c_th_02_t3 = 0.25*c_th_02_t3
    v_c_th_02_t4 = 0.25*c_th_02_t4
    #   Duplicates P1
    s_th_02_t11 = -s_th_0*s_th_2; s_th_02_t12 = s_th_0*s_th_2; s_th_124PI = s_th_1*s_th_2*s_th_4PI; s_th_0_c_th_2 = s_th_0*c_th_2
    #   Duplicates P2
    aux_id_0 = s_th_1*c_th_2*c_th_3PI2; aux_id_1 = s_th_3PI2*c_th_1; aux_id_2 = aux_id_0 + aux_id_1
    aux_id_3 = s_th_1*s_th_3PI2*c_th_0; aux_id_4 = (s_th_02_t11 + v_c_th_02_t2 + v_c_th_02_t1 + v_c_th_02_t4 + v_c_th_02_t3)
    aux_id_33 = (s_th_0_c_th_2 + v_s_th_02_t2 + v_s_th_02_t1 - v_s_th_02_t4 + v_s_th_02_t3)
    aux_id_5 = aux_id_33*s_th_4PI; aux_id_6 = aux_id_4*c_th_3PI2; aux_id_7 = (-aux_id_4*s_th_3PI2 - s_th_1*c_th_0*c_th_3PI2)*s_th_5; aux_id_8 = aux_id_6 - aux_id_3
    aux_id_9 = aux_id_33*c_th_4PI; aux_id_10 = (s_th_02_t12 - v_c_th_02_t2 - v_c_th_02_t1 - v_c_th_02_t4 - v_c_th_02_t3); aux_id_11 = (aux_id_10*c_th_3PI2 + aux_id_3)
    aux_id_12 = (aux_id_11*c_th_4PI - aux_id_5); aux_id_13 = aux_id_10*s_th_3PI2 - s_th_1*c_th_0*c_th_3PI2; aux_id_14 = (aux_id_13)*c_th_5
    aux_id_15 = (s_th_2*c_th_0 - v_s_th_02_t2 + v_s_th_02_t1 + v_s_th_02_t4 + v_s_th_02_t3)*c_th_3PI2; aux_id_23 = s_th_01_t1*s_th_3PI2
    aux_id_16 = (s_th_2*c_th_0 - v_s_th_02_t2 + v_s_th_02_t1 + v_s_th_02_t4 + v_s_th_02_t3)*s_th_3PI2; aux_id_17 = (-aux_id_15 + aux_id_23)*c_th_4PI
    aux_id_18 = (-c_th_0*c_th_2 + v_c_th_02_t2 - v_c_th_02_t1 + v_c_th_02_t4 - v_c_th_02_t3)*s_th_4PI; aux_id_19 = (aux_id_16 + s_th_01_t1*c_th_3PI2)
    aux_id_20 = aux_id_19*s_th_5; aux_id_21 = -aux_id_17 + aux_id_18; aux_id_22 = (aux_id_21)*s_th_5; aux_id_24 = (aux_id_2)*c_th_4PI; aux_id_25 = -aux_id_24 - s_th_124PI
    aux_id_26 = (aux_id_25)*c_th_5; aux_id_27 = (aux_id_25)*s_th_5; aux_id_28 = -s_th_1*s_th_3PI2*c_th_2 + c_th_1*c_th_3PI2; aux_id_29 = (aux_id_28)*s_th_5; aux_id_30 = (aux_id_28)*c_th_5
    aux_id_31 = (aux_id_2)*s_th_4PI - s_th_1*s_th_2*c_th_4PI; aux_id_32 = aux_id_10*c_th_3PI2 + aux_id_3; aux_id_34 = aux_id_32*c_th_4PI; aux_id_35 = (-aux_id_34 + aux_id_5)
    aux_id_36 = (c_th_0*c_th_2 - v_c_th_02_t2 + v_c_th_02_t1 - v_c_th_02_t4 + v_c_th_02_t3); aux_id_37 = s_th_0*s_th_1*s_th_3PI2; aux_id_38 = -(aux_id_0 + aux_id_1)*c_th_4PI
    aux_id_39 = s_th_0*s_th_1*c_th_3PI2

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = -((-(aux_id_8)*c_th_4PI - aux_id_5)*c_th_5 + aux_id_7)*c_th_6 - ((aux_id_8)*s_th_4PI - aux_id_9)*s_th_6
    T[0,1] = (((-aux_id_6 + aux_id_3)*c_th_4PI - aux_id_5)*c_th_5 + aux_id_7)*s_th_6 - ((aux_id_4*c_th_3PI2 - aux_id_3)*s_th_4PI - aux_id_9)*c_th_6
    T[0,2] = -aux_id_12*s_th_5 + aux_id_14
    T[0,3] = 0.036*aux_id_35*s_th_5 - 0.027*aux_id_35*c_th_5 + 0.027*(aux_id_13)*s_th_5 + 0.036*(aux_id_13)*c_th_5 - 0.027*aux_id_34 + 0.265*aux_id_10*s_th_3PI2 - 0.0405*aux_id_10*c_th_3PI2 + 0.027*aux_id_5 - 0.0405*s_th_02_t12 - 0.0405*aux_id_3 - 0.265*s_th_1*c_th_0*c_th_3PI2 + 0.251500010453034*s_th_1*c_th_0 + 0.03*c_th_0*c_th_1 - 0.03*c_th_0 + 0.010125*c_th_02_t2 + 0.0101250012797019*c_th_02_t1 + 0.010125*c_th_02_t4 + 0.0101249986747384*c_th_02_t3
    T[1,0] = (-(aux_id_17 + aux_id_36*s_th_4PI)*c_th_5 + aux_id_20)*c_th_6 + ((-aux_id_15 + aux_id_23)*s_th_4PI - aux_id_36*c_th_4PI)*s_th_6
    T[1,1] = (-(aux_id_21)*c_th_5 - aux_id_20)*s_th_6 + ((-aux_id_15 + aux_id_23)*s_th_4PI + (-c_th_0*c_th_2 + v_c_th_02_t2 - v_c_th_02_t1 + v_c_th_02_t4 - v_c_th_02_t3)*c_th_4PI)*c_th_6
    T[1,2] = aux_id_22 - aux_id_19*c_th_5
    T[1,3] = 0.036*(-(-aux_id_15 + aux_id_37)*c_th_4PI + aux_id_18)*s_th_5 - 0.027*(-(-aux_id_15 + aux_id_37)*c_th_4PI + aux_id_18)*c_th_5 - 0.027*(aux_id_16 + aux_id_39)*s_th_5 - 0.036*(aux_id_16 + aux_id_39)*c_th_5 + 0.027*(aux_id_15 - aux_id_37)*c_th_4PI - 0.265*aux_id_16 + 0.0405*aux_id_15 - 0.027*aux_id_36*s_th_4PI - 0.0405*aux_id_37 - 0.265*aux_id_39 + 0.251500010453034*s_th_0*s_th_1 + 0.03*s_th_0*c_th_1 - 0.03*s_th_0 + 0.0405*s_th_2*c_th_0 - 0.010125*s_th_02_t2 + 0.010125*s_th_02_t1 + 0.010125*s_th_02_t4 + 0.010125*s_th_02_t3
    T[2,0] = (aux_id_26 + aux_id_29)*c_th_6 + (aux_id_31)*s_th_6
    T[2,1] = -(aux_id_26 + aux_id_29)*s_th_6 + (aux_id_31)*c_th_6
    T[2,2] = aux_id_27 - aux_id_30
    T[2,3] = 0.036*(aux_id_38- s_th_124PI)*s_th_5 - 0.027*(aux_id_38- s_th_124PI)*c_th_5 - 0.027*(aux_id_28)*s_th_5 - 0.036*(aux_id_28)*c_th_5 - 0.027*(aux_id_0 + aux_id_1)*c_th_4PI - 0.027*s_th_124PI + 0.265*s_th_1*s_th_3PI2*c_th_2 - 0.0405*aux_id_0 - 0.0405*s_th_1*c_th_2 - 0.03*s_th_1 - 0.0405*aux_id_1 - 0.265*c_th_1*c_th_3PI2 + 0.251500010453034*c_th_1 + 0.1
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def __FKF_EPSON_LS3_B401S(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.List[tp.List[float]]:
    """
    Description:
        Calculation of forward kinematics using a fast method for the Epson LS3 B401S robotic arm.

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
    
    """
    Description:
        Abbreviations for individual functions. Used to speed up calculations.
    """
    # Angles:
    th_01 = theta[0] + theta[1]; th_013 = th_01 - theta[3]
    # Sine / Cosine functions:
    c_th_013 = np.cos(th_013); s_th_013 = np.sin(th_013)

    # Computation of the homogeneous end-effector transformation matrix {T}
    T = np.array(np.identity(4), dtype=np.float64)
    T[0,0] = c_th_013
    T[0,1] = s_th_013
    T[0,2] = 0.0
    T[0,3] = 0.225*np.cos(theta[0]) + 0.175*np.cos(th_01)
    T[1,0] = s_th_013
    T[1,1] = -c_th_013
    T[1,2] = 0.0
    T[1,3] = 0.225*np.sin(theta[0]) + 0.175*np.sin(th_01)
    T[2,0] = 0.0
    T[2,1] = 0.0
    T[2,2] = -1.0
    T[2,3] = theta[2] + 0.144499991167482
    T[3,0] = 0.0
    T[3,1] = 0.0
    T[3,2] = 0.0
    T[3,3] = 1.0

    # T_Base @ T_n @ T_EE
    return Robot_Parameters_Str.T.Base @ T @ Robot_Parameters_Str.T.End_Effector

def FKFast_Solution(theta: tp.List[float], Robot_Parameters_Str: Parameters.Robot_Parameters_Str) -> tp.Tuple[tp.List[float], 
                                                                                                              tp.List[tp.List[float]]]:
    """
    Description:
        Calculation of forward kinematics using the fast method. The function was created by simplifying 
        the modified Denavit-Hartenberg (DH) method.

        Note 1:
            The resulting function shape for each robot was created by manually removing duplicates.

        Note 2:
            The Forward Kinematics fast calculation method works only for defined robot types. But the function can be easily 
            extended to another type of robot.

            Manipulators:
                Universal Robots: UR3
                ABB: IRB 120, IRB 120 with Linear Axis, 
                     IRB 14000 (YuMi)
                Epson: LS3 B401S

    Args:
        (1) theta [Vector<float>]: Desired absolute joint position in radians / meters.
        (2) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.

    Returns:
        (1) parameter [Vector<bool>]: The result is a vector of values with a warning if the limit 
                                      is exceeded. 
                                      Note:
                                        The value in the vector is "True" if the desired absolute 
                                        joint positions are within the limits, and "False" if they 
                                        are not.
        (2) parameter [Matrix<float> 4x4]: Homogeneous end-effector transformation matrix.
    """
        
    return {
        'Universal_Robots_UR3': lambda th, r_param_str: __FKF_Universal_Robots_UR3(th, r_param_str),
        'ABB_IRB_120': lambda th, r_param_str: __FKF_ABB_IRB_120(th, r_param_str),
        'ABB_IRB_120_L_Ax': lambda th, r_param_str: __FKF_ABB_IRB_120_L_Ax(th, r_param_str),
        'ABB_IRB_14000_R': lambda th, r_param_str: __FKF_ABB_IRB_14000_R(th, r_param_str),
        'ABB_IRB_14000_L': lambda th, r_param_str: __FKF_ABB_IRB_14000_L(th, r_param_str),
        'EPSON_LS3_B401S': lambda th, r_param_str: __FKF_EPSON_LS3_B401S(th, r_param_str)
    }[Robot_Parameters_Str.Name](theta, Robot_Parameters_Str)