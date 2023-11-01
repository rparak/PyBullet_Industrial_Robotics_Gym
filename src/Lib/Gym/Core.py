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

# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyBullet (Real-Time Physics Simulation) [pip3 install pybullet]
import pybullet as pb
import pybullet_data
# Time (Time access and conversions)
import time
# Custom Lib.:
#   ../Lib/Parameters/Robot
import Lib.Parameters.Robot
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Lib/Transformation/Core
from Lib.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls, Get_Translation_Matrix
#   ../Lib/Kinematics/Core
import Lib.Kinematics.Core as Kinematics

"""
Description:
    Initialization of constants.
"""
# Gravitational Constant.
CONST_GRAVITY = 9.81

class Robot_Cls(object):
    """
    Description:
        A class for working with a robotic arm object in a PyBullet environment.

    Initialization of the Class:
        Args:
            (1) Robot_Parameters_Str [Robot_Parameters_Str(object)]: The structure of the main parameters of the robot.
            (2) urdf_file_path [string]: The specified path of the robotic structure file with an extension '*.urdf'.
                                            Note:
                                                urdf - Unified Robotics Description Format
            (3) properties [Dictionary {'Enable_GUI': int, 'fps': int, 'External_Base': None or string,
                                        'Camera': {'Yaw': float, 
                                                   'Pitch': float, 
                                                   'Distance': float, 
                                                   'Position': Vector<float> 1x3}}]: The properties of the PyBullet environment.
                                                                                        Note:
                                                                                            'Enable_GUI': Enable/disable the PyBullet explorer view.
                                                                                            'fps': The FPS (Frames Per Seconds) value.
                                                                                            'External_Base: The specified path of the robotic structure 
                                                                                                            base, if it exists. If not, set the value 
                                                                                                            to "None".
                                                                                            'Camera': Camera parameters. For more information, please see 
                                                                                                      the 'Get_Camera_Parameters' function of the class.

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = Lib.Parameters.Robot.ABB_IRB_120_Str
                #   The properties of the PyBullet environment.
                env_properties = {'Enable_GUI': 0, 'fps': 100, 'External_Base': None,
                                  'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance':1.3, 
                                             'Position': [0.05, -0.10, 0.06]}}

                # Initialization of the class.
                Cls = Robot_Cls(Robot_Parameters_Str, f'../Robots/{Robot_Parameters_Str.Name}/{Robot_Parameters_Str.Name}.urdf',
                                env_properties)

            Features:
                # Properties of the class.
                Cls.Theta_0; Cls.T_EE

                # Functions of the class.
                Cls.Set_Absolute_Joint_Position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 100.0, 0.0, 1.0)
    """

    def __init__(self, Robot_Parameters_Str: Lib.Parameters.Robot.Robot_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:
        # << PRIVATE >> #
        self.__Robot_Parameters_Str = Robot_Parameters_Str
        self.__external_object = []
        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = Lib.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=self.__delta_time)

        # Set the parameters of the PyBullet environment.
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

        # Get the translational and rotational part from the transformation matrix.
        p = self.__Robot_Parameters_Str.T.Base.p.all(); q = self.__Robot_Parameters_Str.T.Base.Get_Rotation('QUATERNION')
    
        if properties['External_Base'] != None:
            # Load a physics model of the robotic structure base.
            base_id = pb.loadURDF(properties['External_Base'], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 
                                 useFixedBase=True)
            
            # Disable all collisions of the object
            pb.setCollisionFilterGroupMask(base_id, -1, 0, 0)

            # Load a physics model of the robotic structure.
            self.__robot_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useFixedBase=True, 
                                        flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
            
            # Enable collision detection between specific pairs of links.
            pb.setCollisionFilterPair(self.__robot_id, base_id, -1,-1, 1)
        else:
            # Load a physics model of the robotic structure.
            self.__robot_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useFixedBase=True, 
                                        flags=pb.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)
        
        # Obtain the indices of the movable parts of the robotic structure.
        self.__theta_index = []
        for i in range(pb.getNumJoints(self.__robot_id)):
            info = pb.getJointInfo(self.__robot_id , i)
            if info[2] in [pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC]:
                self.__theta_index.append(i)

    def __Set_Env_Parameters(self, enable_gui: int, camera_parameters: tp.Dict) -> None:
        """
        Description:
            A function to set the parameters of the PyBullet environment.

        Args:
            (1) enable_gui [int]: Enable/disable the PyBullet explorer view.
            (2) camera_parameters [Dictionary {'Yaw': float, 'Pitch': float, 'Distance': float, 
                                               'Position': Vector<float> 1x3}]: The parameters of the camera.
                                                                                    Note:
                                                                                        'Yaw': Yaw angle of the camera.
                                                                                        'Pitch': Pitch angle of the camera.
                                                                                        'Distance': Distance between the camera 
                                                                                                    and the camera target.
                                                                                        'Position': Camera position in Cartesian 
                                                                                                    world space coordinates. 
        """

        # Connect to the physics simulation and create an environment with additional properties.
        pb.connect(pb.GUI, options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')
        pb.setTimeStep(self.__delta_time)
        pb.setRealTimeSimulation(0)
        pb.resetSimulation()
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0.0, 0.0, -CONST_GRAVITY)

        # Set the parameters of the camera.
        pb.resetDebugVisualizerCamera(cameraYaw=camera_parameters['Yaw'], cameraPitch=camera_parameters['Pitch'], cameraDistance=camera_parameters['Distance'], 
                                      cameraTargetPosition=camera_parameters['Position'])
        
        # Configure settings for the built-in OpenGL visualizer.
        pb.configureDebugVisualizer(pb.COV_ENABLE_RENDERING, 1)
        pb.configureDebugVisualizer(pb.COV_ENABLE_SHADOWS, 1)
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, enable_gui)
        pb.configureDebugVisualizer(pb.COV_ENABLE_MOUSE_PICKING, 0)

        # Load a physics model of the plane.
        plane_id = pb.loadURDF('/../../../URDFs/Primitives/Plane/Plane.urdf', globalScaling=0.20, useMaximalCoordinates=True, useFixedBase=True)
        #   Change the texture of the loaded object.
        pb.changeVisualShape(plane_id, -1, textureUniqueId=pb.loadTexture('/../../../Textures/Plane.png'))
        pb.changeVisualShape(plane_id, -1, rgbaColor=[0.55, 0.55, 0.55, 0.95])

    @property
    def is_connected(self) -> bool:
        """
        Description:
            Information about a successful connection to the physical server.

        Returns:
            (1) parameter [bool]: The result is 'True' if it is connected, 'False' if it is not.
        """

        return pb.isConnected()
    
    @property
    def Theta_0(self) -> tp.List[float]:
        """
        Description:
            Get the zero (home) absolute position of the joint in radians/meter.

        Returns:
            (1) parameter [Vector<float>]: Zero (home) absolute joint position in radians / meters.
        """
                
        return self.__Robot_Parameters_Str.Theta.Zero
    
    @property
    def Theta(self) -> tp.List[float]: 
        """
        Description:
            Get the absolute positions of the robot's joints.

        Returns:
            (1) parameter [Vector<float> 1xn]: Current absolute joint position in radians / meters.
                                                Note:
                                                    Where n is the number of joints.
        """
                
        theta_out = np.zeros(self.__Robot_Parameters_Str.Theta.Zero.size, 
                             dtype=np.float64)
        for i, th_index in enumerate(self.__theta_index):
            theta_out[i] = pb.getJointState(self.__robot_id, th_index)[0]

        return theta_out
    
    @property
    def T_EE(self) -> tp.List[tp.List[float]]:
        """
        Description:
            Get the homogeneous transformation matrix of the robot end-effector.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of the End-Effector.
        """
                
        return Kinematics.Forward_Kinematics(self.Theta, 'Fast', self.__Robot_Parameters_Str)[1]

    @property
    def Get_Camera_Parameters(self) -> tp.Dict:
        """
        Description:
            Obtain the camera's parameters.

            Note:
                The obtained parameters can be used as one of the input properties of the class.

        Returns:
            (1) parameter [Dictionary {'Yaw': float, 'Pitch': float, 'Distance': float, 
                                       'Position': Vector<float> 1x3}]: The parameters of the camera.
                                                                            Note:
                                                                                'Yaw': Yaw angle of the camera.
                                                                                'Pitch': Pitch angle of the camera.
                                                                                'Distance': Distance between the camera 
                                                                                            and the camera target.
                                                                                'Position': Camera position in Cartesian 
                                                                                            world space coordinates.
        """

        parameters = pb.getDebugVisualizerCamera()

        return {'Yaw': parameters[8], 'Pitch': parameters[9], 'Distance': parameters[10], 
                'Position': parameters[11]}
    
    def __Step(self) -> None:
        """
        Description:
            A function to perform all the actions in a single forward dynamics 
            simulation step extended with a time step value.
        """

        pb.stepSimulation()

        # The time to approximate and update the state of the dynamic system.
        time.sleep(self.__delta_time)

    def Disconnect(self) -> None:
        """
        Description:
            A function to disconnect the created environment from a physical server.
        """
                
        if self.is_connected == True:
            pb.disconnect()

    def Add_External_Object(self, urdf_file_path: str, T: HTM_Cls, color: tp.Union[None, tp.List[float]], scale: float, 
                            fixed_position: bool, enable_collision: bool) -> None:
        """
        Description:
            A function to add external objects with the *.urdf extension to the PyBullet environment.

        Args:
            (1) urdf_file_path [string]: The specified path of the object file with the extension '*.urdf'.
            (2) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the object.
            (3) color [Vector<float> 1x4]: The color of the object.
                                            Note:
                                                Format: rgba(red, green, blue, alpha)
            (4) scale [float]: The scale factor of the object.
            (5) fixed_position [bool]: Information about whether the position of the object should 
                                       be fixed (static) or dynamic.
            (6) enable_collision [bool]: Information on whether or not the object is to be exposed 
                                         to collisions
        """

        # Get the translational and rotational part from the transformation matrix.
        p = T.p.all(); q = T.Get_Rotation('QUATERNION')

        # Load a physics model of the object.
        object_id = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], globalScaling=scale, useMaximalCoordinates=False, 
                                useFixedBase=fixed_position)
        #   Store the object ID to the list.
        self.__external_object.append(object_id)

        # Set the properties of the added object.
        #   Color.
        if color is not None:
            pb.changeVisualShape(object_id, linkIndex=-1, rgbaColor=color)
        #   Collision.
        if enable_collision == False:
            pb.setCollisionFilterGroupMask(object_id, -1, 0, 0)

    @staticmethod
    def Test_1():
        pb.addUserDebugLine(lineFromXYZ=[0.5, 0.0, 0.0], lineToXYZ=[0.5, 0.0, 0.5], lineColorRGB=[0.0, 1.0, 0.0], lineWidth=1.0)

    def Remove_All_External_Objects(self) -> None:
        """
        Description:
            A function to remove all models with the *.urdf extension from the PyBullet environment 
            that were added using the 'Add_External_Object' function of the class.
        """

        for _, external_obj in enumerate(self.__external_object):
            pb.removeBody(external_obj)

    def Reset(self, mode: str, theta: tp.Union[None, tp.List[float]] = None) -> bool:
        """
        Description:
            Function to reset the absolute position of the robot joints from the selected mode.

            Note:
                The Zero/Home modes are predefined in the robot structure and the Individual mode is used 
                to set the individual position defined in the function input parameter.

        Args:
            (1) mode [string]: Possible modes to reset the absolute position of the joints.
                                Note:
                                    mode = 'Zero', 'Home' or 'Individual'
            (2) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """
                
        try:
            assert mode in ['Zero', 'Home', 'Individual'] 

            if mode == 'Individual':
                assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size
                
                theta_internal = theta
            else:
                theta_internal = self.Theta_0 if mode == 'Zero' else self.__Robot_Parameters_Str.Theta.Home

            for i, (th_i, th_i_limit, th_index) in enumerate(zip(theta_internal, self.__Robot_Parameters_Str.Theta.Limit, 
                                                                 self.__theta_index)):
                if th_i_limit[0] <= th_i <= th_i_limit[1]:
                    # Reset the state (position) of the joint.
                    pb.resetJointState(self.__robot_id, th_index, th_i) 

                else:
                    print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                    return False
                
            return True

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            if mode not in ['Zero', 'Home', 'Individual']:
                print('[ERROR] Incorrect reset mode selected. The selected mode must be chosen from the three options (Zero, Home, Individual).')
            if self.__Robot_Parameters_Str.Theta.Zero.size != theta.size:
                print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Set_Absolute_Joint_Position(self, theta: tp.List[float], force: float, t_0: float, t_1: float) -> bool:
        """
        Description:
            Set the absolute position of the robot joints.

            Note:
                To use the velocity control of the robot's joint, it is necessary to change the input 
                parameters of the 'setJointMotorControl2' function from position to:

                    pb.setJointMotorControl2(self.__robot_id, th_index, pb.VELOCITY_CONTROL, 
                                             targetVelocity=th_v_i, force=force),

                and get the velocity from polynomial trajectories.

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters.
                                            Note:
                                                Where n is the number of joints.
            (2) force [float]: The maximum motor force used to reach the target value.
            (3) t_0 [float]: Animation start time in seconds.
            (4) t_1 [float]: Animation stop time in seconds.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """
                
        try:
            assert self.__Robot_Parameters_Str.Theta.Zero.size == theta.size

            # Generation of multi-axis position trajectories from input parameters.
            theta_arr = []
            for _, (th_actual, th_desired) in enumerate(zip(self.Theta, theta)):
                (theta_arr_i, _, _) = self.__Polynomial_Cls.Generate(th_actual, th_desired, 0.0, 0.0, 0.0, 0.0,
                                                                     t_0, t_1)
                theta_arr.append(theta_arr_i)

            for _, theta_arr_i in enumerate(np.array(theta_arr, dtype=np.float64).T):
                for i, (th_i, th_i_limit, th_index) in enumerate(zip(theta_arr_i, self.__Robot_Parameters_Str.Theta.Limit, 
                                                                     self.__theta_index)): 
                    if th_i_limit[0] <= th_i <= th_i_limit[1]:
                        # Control of the robot's joint positions.
                        pb.setJointMotorControl2(self.__robot_id, th_index, pb.POSITION_CONTROL, targetPosition=th_i, 
                                                 force=force)
                    else:
                        print(f'[WARNING] The desired input joint {th_i} in index {i} is out of limit.')
                        return False

                # Update the state of the dynamic system.
                self.__Step()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Set_TCP_Position(self, T: tp.List[tp.List[float]], mode: str, parameters: tp.Dict = None) -> bool:
        """
        Description:
            Set the TCP (tool center point) of the robot.

        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the desired TCP position.
            (2) mode [string]: The name of the mode to be used to perform the transformation.
                                Note:
                                    mode = 'Reset' or 'Motion'
            (3) parameters [Dictionary {'force': float, 't_0': float, 't_1': float}]: The parameters of the 'Motion' mode. If the mode is equal
                                                                                      to 'Reset', the parameters will be equal to 'None'.
                                                                                        Note:
                                                                                            'force': The maximum motor force used to reach the target value.
                                                                                            't_0': Animation start time in seconds.
                                                                                            't_1': Animation stop time in seconds.

        Returns:
            (1) parameter [bool]: The result is 'True' if the robot is in the desired position,
                                  and 'False' if it is not.
        """ 

        try:
            assert mode in ['Reset', 'Motion']

            if isinstance(T, (list, np.ndarray)):
                T = HTM_Cls(T, np.float64)

            # Get the translational and rotational part from the transformation matrix.
            p = T.p.all(); q = T.Get_Rotation('QUATERNION')

            # A function to compute the inverse kinematics (IK) using the Damped Least Squares (DLS) method.
            theta = np.array(pb.calculateInverseKinematics(bodyUniqueId=self.__robot_id, endEffectorLinkIndex=self.__theta_index[-1], 
                                                           targetPosition=p, targetOrientation=[q.x, q.y, q.z, q.w], 
                                                           lowerLimits=self.__Robot_Parameters_Str.Theta.Limit[:, 0], upperLimits=self.__Robot_Parameters_Str.Theta.Limit[:, 1], 
                                                           restPoses=self.Theta, jointDamping=[0.1]*self.__Robot_Parameters_Str.Theta.Zero.size, 
                                                           solver=pb.IK_DLS), dtype=np.float64)

            if mode == 'Reset':
                return self.Reset('Individual', theta)
            else:
                return self.Set_Absolute_Joint_Position(theta, parameters['force'], parameters['t_0'], parameters['t_1'])

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] An incorrect name of the mode was selected for the ...')
