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
# OS (Operating system interfaces)
import os
# Custom Lib.: 
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot
#       ../RoLE/Trajectory/Utilities
import RoLE.Trajectory.Utilities
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core as Kinematics
#       ../RoLE/Primitives/Core
from RoLE.Primitives.Core import Box_Cls, Point_Cls
#       ../RoLE/Collider/Utilities
from RoLE.Collider.Utilities import Get_Min_Max
#       ../RoLE/Primitives/Core
from RoLE.Collider.Core import AABB_Cls
#   Gym
#       ../Gym/Utilities
import Gym.Utilities

"""
Description:
    Initialization of constants.
"""
# Gravitational Constant.
CONST_GRAVITY = 9.81
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

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
            (3) properties [Dictionary {'Enable_GUI': int, 'fps': int, 'External_Base': None or string, 'Env_ID': int,
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
                                                                                            'Env_ID': The identification number (ID) of the environment.
                                                                                                        Note:
                                                                                                            For more information, see the 
                                                                                                            script ../Configuration/Environment.py.
                                                                                            'Camera': Camera parameters. For more information, please see 
                                                                                                      the 'Get_Camera_Parameters' function of the class.

        Example:
            Initialization:
                # Assignment of the variables.
                #   Example for the ABB IRB 120 robot.
                Robot_Parameters_Str = RoLE.Parameters.Robot.ABB_IRB_120_Str
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

    def __init__(self, Robot_Parameters_Str: RoLE.Parameters.Robot.Robot_Parameters_Str, urdf_file_path: str, properties: tp.Dict) -> None:
        # << PRIVATE >> #
        self.__Robot_Parameters_Str = Robot_Parameters_Str
        self.__external_object = {}
        # Time step.
        self.__delta_time = 1.0/np.float64(properties['fps'])

        # Initialization of the class to generate trajectory.
        self.__Polynomial_Cls = RoLE.Trajectory.Utilities.Polynomial_Profile_Cls(delta_time=self.__delta_time)

        # Set the parameters of the PyBullet environment.
        self.__Set_Env_Parameters(properties['Enable_GUI'], properties['Camera'])

        # Get the translational and rotational part from the transformation matrix.
        p = self.__Robot_Parameters_Str.T.Base.p.all(); q = self.__Robot_Parameters_Str.T.Base.Get_Rotation('QUATERNION')

        if properties['External_Base'] != None:
            # Load a physics model of the robotic structure base.
            base_id = pb.loadURDF(properties['External_Base'], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], 
                                 useFixedBase=True)
            
            # Disable all collisions of the object.
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

        # Load an auxiliary model of the robotic structure, which is represented as a 'ghost'.
        self.__robot_id_ghost = pb.loadURDF(urdf_file_path, p, [q.x, q.y, q.z, q.w], useMaximalCoordinates=False, 
                                          useFixedBase=True)
        #   Disable collision of the robot base.
        pb.setCollisionFilterGroupMask(self.__robot_id_ghost, -1, 0, 0)
        #   Change the texture of the robot base.
        pb.changeVisualShape(self.__robot_id_ghost, linkIndex=-1, rgbaColor=[0.0, 0.55, 0.0, 0.0])
        #   Disable dynamic parameters of the robot base.
        pb.changeDynamics(self.__robot_id_ghost, linkIndex=-1, linearDamping=0, angularDamping=0, jointDamping=0, mass=0)

        # Obtain the indices of the movable parts of the robotic structure.
        self.__theta_index = []
        for i in range(pb.getNumJoints(self.__robot_id)):
            info = pb.getJointInfo(self.__robot_id , i)
            if info[2] in [pb.JOINT_REVOLUTE, pb.JOINT_PRISMATIC]:
                self.__theta_index.append(i)

            # Set the properties of the auxiliary robot structure.
            #   Disable all collisions of the object.
            pb.setCollisionFilterGroupMask(self.__robot_id_ghost, i, 0, 0)
            #   Change the texture of the object.
            pb.changeVisualShape(self.__robot_id_ghost, linkIndex=i, rgbaColor=[0.0, 0.75, 0.0, 0.0])
            #   Disable dynamic parameters of the object.
            pb.changeDynamics(self.__robot_id_ghost, linkIndex=i, linearDamping=0, angularDamping=0, jointDamping=0, mass=0)

        # Obtain the structure of the main parameters of the environment for the defined robotic arm.
        self.__Env_Structure = Gym.Utilities.Get_Environment_Structure(self.__Robot_Parameters_Str.Name, properties['Env_ID'])
        #   Add the cube of the search (configuration) space and get the vertices of the defined cube.
        self.__vertices_C_search = Gym.Utilities.Add_Wireframe_Cuboid(self.__Env_Structure.C.Search.T, self.__Env_Structure.C.Search.Size, 
                                                                      self.__Env_Structure.C.Search.Color, 1.0)
        #   Add the cube of the target (configuration) space and get the vertices of the defined cube.
        self.__vertices_C_target = Gym.Utilities.Add_Wireframe_Cuboid(self.__Env_Structure.C.Target.T, self.__Env_Structure.C.Target.Size, 
                                                                      self.__Env_Structure.C.Target.Color, 1.0)
        
        # Represent the search (configuration) space as Axis-aligned Bounding Boxes (AABB).
        self.__AABB_C_search = AABB_Cls(Box_Cls([0.0, 0.0, 0.0], self.__Env_Structure.C.Search.Size))
        self.__AABB_C_search.Transformation(self.__Env_Structure.C.Search.T)
        #   Initialize a point that will be used to check whether the homogeneous transformation matrix 
        #   of the end-effector is inside the search (configuration) space or not.
        self.__P_EE = Point_Cls([0.0, 0.0, 0.0])

        # Get the home absolute joint positions of a specific environment for a defined robotic arm.
        Robot_Parameters_Str.Theta.Home = Gym.Utilities.Get_Robot_Structure_Theta_Home(self.__Robot_Parameters_Str.Name, properties['Env_ID'])

        # Get the homogeneous transformation matrix of the robot end-effector in the 'Home' position.
        T_Home = Kinematics.Forward_Kinematics(self.__Robot_Parameters_Str.Theta.Home, 'Fast', 
                                               self.__Robot_Parameters_Str)[1]
        #   Get the rotational part from the transformation matrix.
        self.__q_Home = T_Home.Get_Rotation('QUATERNION').all()

        # Add an external collision object with the cuboid wireframe to the environment.
        if self.__Env_Structure.Collision_Object != None:
            self.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/{self.__Env_Structure.Collision_Object.Type}/{self.__Env_Structure.Collision_Object.Type}.urdf', 
                                     f'{self.__Env_Structure.Collision_Object.Type}_Collision', self.__Env_Structure.Collision_Object.T, self.__Env_Structure.Collision_Object.Color,
                                     self.__Env_Structure.Collision_Object.Scale, True)
            _ = Gym.Utilities.Add_Wireframe_Cuboid(self.__Env_Structure.Collision_Object.T, 3 * [self.__Env_Structure.Collision_Object.Scale * 2.0], 
                                                   self.__Env_Structure.Collision_Object.Color[0:3], 1.0)

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
        plane_id = pb.loadURDF(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/Plane/Plane.urdf', globalScaling=0.20, useMaximalCoordinates=True, useFixedBase=True)
        #   Change the texture of the loaded object.
        pb.changeVisualShape(plane_id, -1, textureUniqueId=pb.loadTexture(f'{CONST_PROJECT_FOLDER}/Textures/Plane.png'))
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
            (1) parameter [Vector<float> 1xn]: Zero (home) absolute joint position in radians / meters.
                                                Note:
                                                    Where n is the number of joints.
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
    
    def Get_Configuration_Space_Vertices(self, C_type: str):
        """
        Description:
            Get the vertices of the selected configuration space.

        Args:
            (1) C_type [string]: Type of the configuration space.
                                    Note:
                                        C_type = 'Search' or 'Target'

        Returns:
            (1) parameter [Vector<float> 8x3]: Vertices of the selected configuration space.
        """

        try:
            assert C_type in ['Search', 'Target']

            if C_type == 'Search':
                return self.__vertices_C_search
            else:
                return self.__vertices_C_target

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect configuration type selected. The selected mode must be chosen from the two options (Search, Target).')

    def Step(self) -> None:
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

    def Add_External_Object(self, urdf_file_path: str, name: str, T: HTM_Cls, color: tp.Union[None, tp.List[float]], 
                            scale: float, enable_collision: bool) -> None:
        """
        Description:
            A function to add external objects with the *.urdf extension to the PyBullet environment.

        Args:
            (1) urdf_file_path [string]: The specified path of the object file with the extension '*.urdf'.
            (2) name [string]: The name of the object.
            (3) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the object.
            (4) color [Vector<float> 1x4]: The color of the object.
                                            Note:
                                                Format: rgba(red, green, blue, alpha)
            (5) scale [float]: The scale factor of the object.
            (6) enable_collision [bool]: Information on whether or not the object is to be exposed 
                                         to collisions.
        """

        # Get the translational and rotational part from the transformation matrix.
        p = T.p.all(); q = T.Get_Rotation('QUATERNION')

        # Load a physics model of the object.
        #   Note:
        #       Set the object position to 'Zero'.
        object_id = pb.loadURDF(urdf_file_path, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0], globalScaling=scale, useMaximalCoordinates=False, 
                                useFixedBase=True)
        #   Store the object ID and object name into the dictionary.
        self.__external_object[name] = object_id

        # Get the minimum and maximum X, Y, and Z values from the AABB coordinates of the object.
        (min_AABB, max_AABB) = pb.getAABB(object_id)

        # Set the object position to the desired position defined by the function 
        # input parameters.
        pb.resetBasePositionAndOrientation(object_id, p, [q.x, q.y, q.z, q.w])
        # Disable all collisions of the object.
        #   Note:
        #       Collisions will be solved internally.
        pb.setCollisionFilterGroupMask(object_id, -1, 0, 0)

        # Set the properties of the added object.
        #   Color.
        if color is not None:
            pb.changeVisualShape(object_id, linkIndex=-1, rgbaColor=color)
        #   Collision.
        if enable_collision == True:
            # Add a collider (type AABB) as a part of the robotic arm structure.
            self.__Robot_Parameters_Str.Collider.External[name] = AABB_Cls(Box_Cls([0.0, 0.0, 0.0], 
                                                                                  [max_AABB[0] - min_AABB[0],
                                                                                   max_AABB[1] - min_AABB[1],
                                                                                   max_AABB[2] - min_AABB[2]]))
            # Axis-aligned Bounding Boxe (AABB) transformation according to the input homogeneous 
            # transformation matrix.
            self.__Robot_Parameters_Str.Collider.External[name].Transformation(T)

    def Remove_External_Object(self, name: str) -> None:
        """
        Description:
            A function to remove a specific model with the *.urdf extension from the PyBullet environment
            that was added using the 'Add_External_Object' function of the class.

            Note:
                The function also removes external collider added to the robotic structure.

        Args:
            (1) name [string]: The name of the object.
        """

        if name in self.__external_object.keys():
            pb.removeBody(self.__external_object[name])
            del self.__external_object[name]

        if name in self.__Robot_Parameters_Str.Collider.External.keys():
            del self.__Robot_Parameters_Str.Collider.External[name]

    def Remove_All_External_Objects(self) -> None:
        """
        Description:
            A function to remove all models with the *.urdf extension from the PyBullet environment 
            that were added using the 'Add_External_Object' function of the class.

            Note:
                The function also removes external colliders added to the robotic structure.
        """

        for _, external_obj in enumerate(self.__external_object.values()):
            pb.removeBody(external_obj)

        self.__external_object = {}; self.__Robot_Parameters_Str.Collider.External = {}

    def Generate_Random_T_EE(self, C_type: str, visibility: bool) -> tp.List[tp.List[float]]:
        """
        Description:
            A function that generates the homogeneous transformation matrix of a random end-effector 
            position within the defined configuration space.

        Args:
            (1) C_type [string]: Type of the configuration space.
                                    Note:
                                        C_type = 'Search' or 'Target'
            (2) visibility [bool]: Information about whether the random point will be displayed 
                                   in the PyBullet environment or not.

        Returns:
            (1) parameter [Matrix<float> 4x4]: Homogeneous transformation matrix of a random end-effector position 
                                               within the defined configuration space.
        """

        try:
            assert C_type in ['Search', 'Target'] 

            if C_type == 'Search':
                # Get the minimum and maximum X, Y, Z values of the input vertices.
                (min_vec3, max_vec3) = Get_Min_Max(self.__vertices_C_search)

            else:
                # Get the minimum and maximum X, Y, Z values of the input vertices.
                (min_vec3, max_vec3) = Get_Min_Max(self.__vertices_C_target)
            
            x = np.random.uniform(min_vec3[0], max_vec3[0])
            y = np.random.uniform(min_vec3[1], max_vec3[1])
            z = np.random.uniform(min_vec3[2], max_vec3[2])

            # Obtain the homogeneous transformation matrix of a random end-effector position.
            T = HTM_Cls(None, np.float32).Rotation(self.__q_Home, 'QUATERNION').Translation([x, y, z])

            if visibility == True:
                # Removal of external objects corresponding to a random point.
                self.Remove_External_Object('T_EE_Rand_Viewpoint')
                self.Remove_External_Object('T_EE_Rand_Tolerance')
                
                # Adding external objects corresponding to a random point.
                self.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Rand_Viewpoint', T,
                                         None, 0.3, False)
            return T

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect configuration type selected. The selected mode must be chosen from the two options (Search, Target).')

    def __Reset_Ghost_Structure(self, theta: tp.List[float], visibility: bool, color: tp.Union[None, tp.List[float]]) -> None:
        """
        Description:
            Function to reset the absolute position of the auxiliary robot structure, which is represented as a 'ghost'.

        Args:
            (1) theta [Vector<float> 1xn]: Desired absolute joint position in radians / meters. Used only in individual 
                                           mode.
                                            Note:
                                                Where n is the number of joints.
            (2) visibility [bool]: Visibility of the target position as the 'ghost' of the robotic model.
            (3) info [bool]: Information on whether the result was found within the required tolerance.
            (4) color [None or Vector<float> 1x3]: The color of the object.
                                                    Note:
                                                        Format: rgb(red, green, blue)
        """

        alpha = 0.3 if visibility == True else 0.0
        for _, (th_i, th_index) in enumerate(zip(theta, self.__theta_index)):
            # Reset the state (position) of the joint.
            pb.resetJointState(self.__robot_id_ghost, th_index, th_i) 

            # Set the properties of the auxiliary model.
            #   Change the texture of the object.
            pb.changeVisualShape(self.__robot_id_ghost, linkIndex=th_index, rgbaColor=np.append([color], [alpha]))
    
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
                self.Step()

            return True
            
        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print(f'[ERROR] Incorrect number of values in the input variable theta. The input variable "theta" must contain {self.__Robot_Parameters_Str.Theta.Zero.size} values.')

    def Get_Inverse_Kinematics_Solution(self, T: tp.List[tp.List[float]], ik_solver_properties: tp.Dict, enable_ghost: bool) -> tp.Dict[bool, tp.List[float]]:
        """
        Description:
            A function to compute the inverse kinematics (IK) solution of the individual robotic 
            structure using the numerical method.

            Note:
                The function also includes verification to determine if the TCP input position 
                is within the search area.

        Args:
            (1) T [Matrix<float> 4x4]: Homogeneous transformation matrix of the desired TCP position.
            (2) ik_solver_properties [Dictionary {'delta_time': float or None, 'num_of_iteration': float, 
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
            (3) enable_ghost [bool]: Enable visibility of the auxiliary robotic structure, which is represented as a 'ghost'.

        Returns:
            (1) parameter [bool]: The result is 'True' if the inverse kinematics (IK) has a solution, and 'False' if 
                                  it does not.
            (2) parameter [Vector<float> 1xn]: Obtained solution of the absolute positions of the joints in radians / meters.
                                                Note:
                                                    Where n is the number of joints.
        """

        if isinstance(T, (list, np.ndarray)):
            T = HTM_Cls(T, np.float64)

        # Transformation of point position in X, Y, Z axes.
        self.__P_EE.Transformation(T.p.all())

        # Determine if a given point is inside a search area.
        if self.__AABB_C_search.Is_Point_Inside(self.__P_EE) == True:
            # A function to compute the inverse kinematics (IK) using the using the chosen numerical method.
            (info, theta) = Kinematics.Inverse_Kinematics_Numerical(T, self.Theta, 'Levenberg-Marquardt', self.__Robot_Parameters_Str, 
                                                                    ik_solver_properties)
                
            # Check whether the inverse kinematics (IK) has a solution or not.
            #   Conditions:
            #       1\ IK solution within limits.
            #       2\ Collision-free.
            #       3\ No singularities.
            if info['successful'] == True:
                # Check whether a part of the robotic structure collides with external objects.
                (is_external_collision, _) = Kinematics.General.Is_External_Collision(theta, self.__Robot_Parameters_Str)

                if info['is_self_collision'] == False and info['is_close_singularity'] == False \
                    and is_external_collision == False:
                        successful = True
                else:
                    successful = False
            else:
                successful = False

            # Reset the absolute position of the auxiliary robot structure, which is represented as a 'ghost'.
            #   Note:
            #       'Red': Collision.
            #       'Green': No collision.
            if successful:
                self.__Reset_Ghost_Structure(theta, enable_ghost, [0.70, 0.85, 0.60])
            else:
                self.__Reset_Ghost_Structure(theta, enable_ghost, [0.85, 0.60, 0.60])

            return (successful, theta)
        else:
            # Set the color of the 'ghost' structure to 'red' to indicate that something is wrong.
            self.__Reset_Ghost_Structure(self.Theta, enable_ghost, [0.85, 0.60, 0.60])

            return (False, self.Theta)
