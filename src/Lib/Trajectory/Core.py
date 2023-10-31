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
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Lib/Transformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics

"""
Description:
    Some useful references about the field of trajectory generation:
        1. Modern Robotics: Mechanics, Planning, and Control, Kevin M. Lynch and Frank C. Park
        2. Trajectory Planning for Automatic Machines and Robots by Luigi Biagiotti, Claudio Melchiorri
        3. Introduction To Robotics, Mechanics and Control, John J. Craig
""" 

class Multi_Segment_Cls(object):
    """
    Description:
        The specific class for generating the multi-segment trajectory using the two different methods.

    Initialization of the Class:
        Args:
            (1) method [string]: The method of the multi-segment trajectory generation. 
                                    Note:
                                        method = 'Trapezoidal', 'Polynomial'
            (2) delta_time [float]: The difference (spacing) between the time values.

        Example:
            Initialization:
                # Assignment of the variables.
                method = 'Trepezoidal'
                delta_time = 0.01

                # Initialization of the class.
                Cls = Multi_Segment_Cls(method, delta_time)

            Features:
                # Properties of the class.
                Cls.t
                ...
                Cls.N

                # Functions of the class.
                Cls.Generate()
    """

    def __init__(self, method: str, delta_time: float) -> None:
        try:
            assert method in ['Trapezoidal', 'Polynomial']

            # The method of the multi-segment trajectory generation. 
            self.__method = method

            # The difference (spacing) between the time values.
            self.__delta_time = delta_time

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect method name of the class input parameter. The method must be named as "Trapezoidal" or "Polynomial", not as "{method}".')
        
    @property
    def t(self) -> tp.List[float]:
        """
        Description:
            Get evenly spaced time values at the following interval, see below:
                t_0 <= t <= t_f,

                where t_0 is 0.0 and t_f is is the sum of the duration between 
                trajectory segments.

        Returns:
            (1) parameter [Vector<float> 1xn]: Time values.
                                                Note:
                                                    Where n is the number of time values.
        """
                
        return self.__t
    
    @property
    def N(self) -> int:
        """
        Description:
           Get the number of time points of the trajectory.
        
        Returns:
            (1) parameter [int]: Number of time points.
        """
                
        return self.__t.size
    
    @property
    def delta_time(self) -> float:
        """
        Description:
           Get the difference (spacing) between the time values.
        
        Returns:
            (1) parameter [float]: The difference (spacing) between the time values.
        """

        return self.__delta_time

    @property
    def Method(self) -> str:
        """
        Description:
           Get the method of the multi-segment trajectory generation. 
        
        Returns:
            (1) parameter [float]: The method of the multi-segment trajectory generation. 
        """

        return self.__method

    def __Get_Arc_Length(self, s_dot: tp.List[float]) -> float:
        """
        Description:
            Obtain the arc length L(t) of the position part of the trajectory.

            The arc length L(t) is defined by:
                L(t) = \int_{0}^{t} ||s'(t)||_{2} dt.

        Args:
            (1) s_dot [Vector<float> 1xN]: Velocity part of the trapezoidal trajectory.
                                                    Note:
                                                        Where N is the number of time points of the trajectory.

        Returns:
            (1) parameter [float]: The arc length L(t) of the position part of the trajectory.
        """
                
        L = 0.0
        for _, s_dot_i in enumerate(s_dot):
            L += Mathematics.Euclidean_Norm(s_dot_i)

        return L * self.__delta_time
    
    def __Generate_Trapezoidal(self, P: tp.List[tp.List[float]], t_blend: tp.List[float], T: tp.List[float], 
                                                                                          v: tp.List[float]) -> tp.Tuple[tp.List[float], 
                                                                                                                         tp.List[float], 
                                                                                                                         tp.List[float]]:
        """
        Description:
            A function to generate position, velocity, and acceleration of a multi-segment trajectory using 
            the trapezoidal trajectories method.

        Args:
            (1) P [Vector<float> 1xn]: Input control points (waypoints) to be used for trajectory generation.
            (2) t_blend [Vector<float> 1xn]: Duration of the blend phase.
            (3) T [Vector<float> 1xn]: The velocity of each trajectory segment.
            (4) v [Vector<float> 1x(n-1)]: The time of each trajectory segment.

        Returns:
            (1 - 3) parameter [Vector<float> 1xN]: Position, velocity and acceleration trapezoidal trajectory.
                                                    Note:
                                                        Where N is the number of time points of the trajectory.

        Note:
            The variable n equals the number of points.
        """
                
        # Initialization of the class to generate trajectory using using a trapezoidal 
        # profile.
        T_Cls = Lib.Trajectory.Utilities.Trapezoidal_Profile_Cls(self.__delta_time)

        # Phase 1: The trajectory starts.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = T_Cls.Generate(P[0], P[0] + v[0] * t_blend[0], 0.0, v[0], T[0] - t_blend[0], T[0] + t_blend[0])
        self.__t = T_Cls.t
        s = s_tmp; s_dot = s_dot_tmp; s_ddot = s_ddot_tmp

        # Phase 2: The trajectory alternates between linear and blend phases.
        for _, (P_ii, v_i, v_ii, T_i, T_ii, t_blend_i, t_blend_ii) in enumerate(zip(P[1::], v, v[1::], T, T[1::], t_blend, t_blend[1::])):
            # Linear phase.
            (s_tmp, s_dot_tmp, s_ddot_tmp) = T_Cls.Generate(s[-1], s[-1] + v_i*((T_ii - t_blend_ii) - (T_i + t_blend_i)), v_i, v_i, T_i + t_blend_i, T_ii - t_blend_ii)
            self.__t = np.concatenate((self.__t, T_Cls.t), dtype=np.float64)
            s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)

            # Trapezoidal blends phase.
            (s_tmp, s_dot_tmp, s_ddot_tmp) = T_Cls.Generate(s[-1], P_ii + v_ii * t_blend_ii, v_i, v_ii, T_ii - t_blend_ii, T_ii + t_blend_ii)
            self.__t = np.concatenate((self.__t, T_Cls.t), dtype=np.float64)
            s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64) 

        # Phase 3: The trajectory ends.
        #   Linear phase.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = T_Cls.Generate(s[-1], s[-1] + v[-1]*((T[-1] - t_blend[-1]) - (T[-2] + t_blend[-2])), v[-1], v[-1], T[-2] + t_blend[-2], T[-1] - t_blend[-1])
        self.__t = np.concatenate((self.__t, T_Cls.t), dtype=np.float64)
        s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)
        #   Trapezoidal blends phase.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = T_Cls.Generate(s[-1], P[-1], v[-1], 0.0, T[-1] - t_blend[-1], T[-1] + t_blend[-1])
        self.__t = np.concatenate((self.__t, T_Cls.t), dtype=np.float64)
        s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)

        # Release of the object.
        del T_Cls

        return (s, s_dot, s_ddot)

    def __Generate_Polynomial(self, P: tp.List[tp.List[float]], t_blend: tp.List[float], T: tp.List[float], 
                                                                                         v: tp.List[float]) -> tp.Tuple[tp.List[float], 
                                                                                                                        tp.List[float], 
                                                                                                                        tp.List[float]]:
        """
        Description:
            A function to generate position, velocity, and acceleration of a multi-segment trajectory using 
            the polynomial trajectories (degree 5 - quintic) method.

        Args:
            (1) P [Vector<float> 1xn]: Input control points (waypoints) to be used for trajectory generation.
            (2) t_blend [Vector<float> 1xn]: Duration of the blend phase.
            (3) T [Vector<float> 1xn]: The velocity of each trajectory segment.
            (4) v [Vector<float> 1x(n-1)]: The time of each trajectory segment.

        Returns:
            (1 - 3) parameter [Vector<float> 1xN]: Position, velocity and acceleration trapezoidal trajectory.
                                                    Note:
                                                        Where N is the number of time points of the trajectory.

        Note:
            The variable n equals the number of points.
        """
                
        # Initialization of the class to generate trajectory using a linear function.
        L_Cls = Lib.Trajectory.Utilities.Linear_Function_Cls(self.__delta_time)

        # Initialization of the class to generate trajectory using using a polynomial 
        # profile of degree 5 (quintic).
        P_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(self.__delta_time)

        # Phase 1: The trajectory starts.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = P_Cls.Generate(P[0], P[0] + v[0] * t_blend[0], 0.0, v[0], 0.0, 0.0, T[0] - t_blend[0], T[0] + t_blend[0])
        self.__t = P_Cls.t
        s = s_tmp; s_dot = s_dot_tmp; s_ddot = s_ddot_tmp

        # Phase 2: The trajectory alternates between linear and blend phases.
        for _, (P_ii, v_i, v_ii, T_i, T_ii, t_blend_i, t_blend_ii) in enumerate(zip(P[1::], v, v[1::], T, T[1::], t_blend, t_blend[1::])):
            # Linear phase.
            (s_tmp, s_dot_tmp, s_ddot_tmp) = L_Cls.Generate(s[-1], v_i, T_i + t_blend_i, T_ii - t_blend_ii)
            self.__t = np.concatenate((self.__t, L_Cls.t), dtype=np.float64)
            s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)

            # Polynomial blends phase.
            (s_tmp, s_dot_tmp, s_ddot_tmp) = P_Cls.Generate(s[-1], P_ii + v_ii * t_blend_ii, v_i, v_ii, 0.0, 0.0, T_ii - t_blend_ii, T_ii + t_blend_ii)
            self.__t = np.concatenate((self.__t, P_Cls.t), dtype=np.float64)
            s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64) 

        # Phase 3: The trajectory ends.
        #   Linear phase.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = L_Cls.Generate(s[-1], v[-1], T[-2] + t_blend[-2], T[-1] - t_blend[-1])
        self.__t = np.concatenate((self.__t, L_Cls.t), dtype=np.float64)
        s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)
        #   Polynomial blends phase.
        (s_tmp, s_dot_tmp, s_ddot_tmp) = P_Cls.Generate(s[-1], P[-1], v[-1], 0.0, 0.0, 0.0, T[-1] - t_blend[-1], T[-1] + t_blend[-1])
        self.__t = np.concatenate((self.__t, P_Cls.t), dtype=np.float64)
        s = np.concatenate((s, s_tmp), dtype=np.float64); s_dot  = np.concatenate((s_dot, s_dot_tmp), dtype=np.float64); s_ddot = np.concatenate((s_ddot, s_ddot_tmp), dtype=np.float64)

        # Release of the object.
        del L_Cls; del P_Cls

        return (s, s_dot, s_ddot)

    def Generate(self, P: tp.List[tp.List[float]], delta_T: tp.List[float], t_blend: tp.List[float]) -> tp.Tuple[tp.List[float], 
                                                                                                                 tp.List[float], 
                                                                                                                 tp.List[float],
                                                                                                                 tp.List[float],
                                                                                                                 float]:
        """
        Description:
            A function to generate position, velocity, and acceleration of a multi-segment trajectory using 
            the selected method.

        Args:
            (1) P [Vector<float> 1xn]: Input control points (waypoints) to be used for trajectory generation.
            (2) delta_T [Vector<float> 1x(n-1)]: Trajectory duration between control points.
            (3) t_blend [Vector<float> 1xn]: Duration of the blend phase.

        Returns:
            (1 - 3) parameter [Vector<float> 1xN]: Position, velocity and acceleration trapezoidal trajectory.
                                                    Note:
                                                        Where N is the number of time points of the trajectory.
            (4) parameter [Vector<float> 1xn]: The time of each trajectory segment.
                                                Note:
                                                    when the trajectory reaches the control points
            (5) parameter [float]: The arc length L(t) of the position part of the trajectory.
            
        Note:
            The variable n equals the number of points.
        """

        try:
            assert P.size == t_blend.size and P.size == delta_T.size + 1

            # Express the velocity of each trajectory segment.
            v = np.zeros(delta_T.size, dtype=np.float64)
            for i, (P_i, P_ii, delta_T_i) in enumerate(zip(P, P[1::], delta_T)):
                v[i] = (P_ii - P_i)/delta_T_i

            # Express the time of each trajectory segment.
            T = np.zeros(P.size, dtype=np.float64)
            for i, delta_T_i in enumerate(delta_T, start=1):
                T[i] = T[i-1] + delta_T_i

            if self.__method == 'Trapezoidal':
                (s, s_dot, s_ddot) = self.__Generate_Trapezoidal(P, t_blend, T, v)
            else:
                (s, s_dot, s_ddot) = self.__Generate_Polynomial(P, t_blend, T, v)

            return (s, s_dot, s_ddot, T, self.__Get_Arc_Length(s_dot))
        
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect size of function input parameters.')
            print('[ERROR] The expected size of the input parameters must be of the form P(1, n), t(1, n-1) and t_blend(1, n).')
