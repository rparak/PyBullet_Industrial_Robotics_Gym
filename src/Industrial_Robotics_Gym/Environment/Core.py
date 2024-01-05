# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Typing (Support for type hints)
import typing as tp
# OS (Operating system interfaces)
import os
# Gymnasium (Developing and comparing reinforcement learning algorithms) [pip3 install gymnasium]
import gymnasium as gym
# Custom Lib.: 
#   Robotics Library for Everyone (RoLE)
#       ../RoLE/Parameters/Robot
import RoLE.Parameters.Robot as Parameters
#       ../RoLE/Kinematics/Core
import RoLE.Kinematics.Core as Kinematics
#       ../RoLE/Transformation/Core
from RoLE.Transformation.Core import Homogeneous_Transformation_Matrix_Cls as HTM_Cls
#       ../RoLE/Primitives/Core
from RoLE.Primitives.Core import Box_Cls, Point_Cls
#       ../RoLE/Collider/Utilities
from RoLE.Collider.Utilities import Get_Min_Max
#       ../RoLE/Primitives/Core
from RoLE.Collider.Core import AABB_Cls
#   PyBullet
#       ../PyBullet/Core
import PyBullet.Core
#       ../PyBullet/Utilities
import PyBullet.Utilities

"""
Description:
    Initialization of constants.
"""
# Locate the path to the project folder.
CONST_PROJECT_FOLDER = os.getcwd().split('PyBullet_Industrial_Robotics_Gym')[0] + 'PyBullet_Industrial_Robotics_Gym'

class Industrial_Robotics_Gym_Env_Cls(gym.Env):
    """
    Description:
        A class designed to train a specific robotic arm on the 'Reach' task within a pre-defined environment 
        using the Deep Reinforcement Learning (DRL) algorithm.

    Initialization of the Class:
        Args:
            (1) mode [string]: The name of the environment mode.
            (2) Robot_Str [Robot_Str(object)]: The structure of the main parameters of the robot.
            (3) action_step_factor [float]: The reduction of the action step.
            (4) distance_threshold [float]: The threshold distance at which the result will be successfully completed.
            (5) T [Matrix<float, float> 4x4]: Homogeneous transformation matrix of the robot end-effector position 
                                              within the 'Target' configuration space.

        Example:
            # Create the environment that was previously registered using gymnasium.register() within the __init__.py file.
            #   More information can be found in the following script:
            #       ../src/Industrial_Robotics_Gym/__init__.py
            gym_environment = gym.make(Industrial_Robotics_Gym.Utilities.Get_Environment_ID('Ur3-Default-Reach-v0', T=None)

            # Reset the pre-defined environment of the gym.
            #   Note:
            #       Obtain the initial information and observations.
            observations, informations = gym_environment.reset()

            for _ in range(1000):
                # Obtain a random action sample from the entire action space.
                action = gym_environment.action_space.sample()

                # Perform the action within the pre-defined environment and get the new observation space.
                observations, reward, terminated, truncated, informations = gym_environment.step(action)

                # When the reach task process is terminated or truncated, reset the pre-defined gym environment.
                if terminated == True or truncated == True:
                    observations, informations = gym_environment.reset()

            # Disconnect the created environment from a physical server.
            gym_environment.close()
    """

    def __init__(self, mode: str = 'Default', Robot_Str: Parameters = Parameters.Universal_Robots_UR3_Str, action_step_factor: float = 1.0, 
                 distance_threshold: float = 1.0, T: HTM_Cls = None) -> None:
        try:
            assert mode in ['Default', 'Collision-Free']

            super(Industrial_Robotics_Gym_Env_Cls, self).__init__()

            # Express the input variables.
            self.__distance_threshold = np.float32(distance_threshold)
            self.__action_step_factor = np.float32(action_step_factor)
            self.__mode = mode
            self.__T = T

            # Numerical IK Parameters.
            #   The properties of the inverse kinematics solver.
            self.__ik_properties = {'delta_time': 0.2, 'num_of_iteration': 100, 
                                    'tolerance': 1e-30}
        
            # Set the parameters of the environment.
            self.__Set_Env_Parameters(mode, Robot_Str)

            # Get the translational and rotational part from the input transformation matrix.
            if self.__T is not None:
                self.__p_T = T.p.all().astype(np.float32); self.__q_T = T.Get_Rotation('QUATERNION').all().astype(np.float32)

                # Transformation of point position in X, Y, Z axes.
                self.__P_EE.Transformation(self.__p_T)

                # Determine if a given point is inside a search area.
                if self.__AABB_C_search.Is_Point_Inside(self.__P_EE) == False:
                    raise Exception('[ERROR] The input transformation matrix has point outside the "Target" configuration space.')

            # Reset the pre-defined environment of the gym.
            #   Note:
            #       Obtain the initial observations.
            observation, _ = self.reset()

            # Brief description of the action space and the observation space.
            #   The action space is defined by the Cartesian displacement in each x, y and z axis of the robot's end-effector.
            self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3, ), dtype=np.float32)
            #   The observation space is defined by a dictionary with information about the position and linear velocity of the robot 
            #   end-effector, as well as information about the achieved and desired goals.
            self.observation_space = gym.spaces.Dict({'observation': gym.spaces.Box(-1.0, 1.0, shape=observation['observation'].shape, dtype=np.float32),
                                                      'achieved_goal': gym.spaces.Box(-1.0, 1.0, shape=observation['achieved_goal'].shape, dtype=np.float32),
                                                      'desired_goal': gym.spaces.Box(-1.0, 1.0, shape=observation['desired_goal'].shape, dtype=np.float32)})

        except AssertionError as error:
            print(f'[ERROR] Information: {error}')
            print('[ERROR] Incorrect environment mode selected. The selected mode must be chosen from the two options (Default, Collision-Free).')

    def __Set_Env_Parameters(self, mode: str, Robot_Str: Parameters) -> None:
        """
        Description:
            A function to set the parameters of the environment.

        Args:
            (1) mode [string]: The name of the environment mode.
            (2) Robot_Str [Robot_Str(object)]: The structure of the main parameters of the robot.
        """
            
        if 'ABB_IRB_14000' in Robot_Str.Name:
            external_base = f'{CONST_PROJECT_FOLDER}/URDFs/Robots/ABB_IRB_14000_Base/ABB_IRB_14000_Base.urdf'
        else:
            external_base = None

        # The properties of the PyBullet environment. 
        pybullet_env_properties = {'Enable_GUI': True, 'fps': 1000, 
                                   'External_Base': external_base, 'Env_ID': 0 if mode == 'Default' else 1,
                                   'Camera': {'Yaw': 70.0, 'Pitch': -32.0, 'Distance': 1.3, 
                                              'Position': [0.05, -0.10, 0.06]}}
        
        # Initialization of the class to work with a robotic arm object in a PyBullet environment.
        self.__PyBullet_Robot_Cls = PyBullet.Core.Robot_Cls(Robot_Str, f'{CONST_PROJECT_FOLDER}/URDFs/Robots/{Robot_Str.Name}/{Robot_Str.Name}.urdf', 
                                                            pybullet_env_properties)
        
        # Reset the absolute position of the robot joints to the 'Home'.
        self.__PyBullet_Robot_Cls.Reset(mode='Home', enable_ghost=False)

        # Get the homogeneous transformation matrix of the robot end-effector in the 'Home' position.
        self.__T_0 = Kinematics.Forward_Kinematics(Robot_Str.Theta.Home, 'Fast', Robot_Str)[1]
        #   Get the translational and rotational part from the transformation matrix.
        self.__p_0 = self.__T_0.p.all().astype(np.float32); self.__q_0 = self.__T_0.Get_Rotation('QUATERNION').all().astype(np.float32)

        # Obtain the structure of the main parameters of the environment for the defined robotic arm.
        self.__Env_Structure = PyBullet.Utilities.Get_Environment_Structure(Robot_Str.Name, pybullet_env_properties['Env_ID'])

        # Represent the search (configuration) space as Axis-aligned Bounding Boxes (AABB).
        self.__AABB_C_search = AABB_Cls(Box_Cls([0.0, 0.0, 0.0], self.__Env_Structure.C.Search.Size))
        self.__AABB_C_search.Transformation(self.__Env_Structure.C.Search.T)
        #   Initialize a point that will be used to check whether the homogeneous transformation matrix 
        #   of the end-effector is inside the search (configuration) space or not.
        self.__P_EE = Point_Cls([0.0, 0.0, 0.0])

        # Get the vertices of the selected configuration space.
        vertices_C_target = self.__PyBullet_Robot_Cls.Get_Configuration_Space_Vertices('Target')

        # Get the minimum and maximum X, Y, Z values of the input vertices.
        (min_vec3, max_vec3) = Get_Min_Max(vertices_C_target)
        self.__min_vec3 = min_vec3.astype(np.float32)
        self.__max_vec3 = max_vec3.astype(np.float32)

        # Adding external objects corresponding to a random point.
        #   Note:
        #       The size of the sphere corresponds to the threshold distance.
        self.__PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Viewpoint/Viewpoint.urdf', 'T_EE_Rand_Viewpoint', HTM_Cls(None, np.float32),
                                                      None, 0.3, False)
        self.__PyBullet_Robot_Cls.Add_External_Object(f'{CONST_PROJECT_FOLDER}/URDFs/Primitives/Sphere/Sphere.urdf', 'T_EE_Rand_Sphere', HTM_Cls(None, np.float32),
                                                      [0.0, 1.0, 0.0, 0.2], self.__distance_threshold, False)

    @staticmethod
    def __Euclidean_Norm(x: tp.List[float]) -> tp.List[float]:
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
            
        return np.linalg.norm(x, axis=-1)

    def compute_reward(self, p: tp.List[float], p_1: tp.List[float], info: tp.Dict[str, tp.Any] = {}) -> tp.List[float]:
        """
        Description:
            Calculate the reward function using the 'Dense' method.

            Note:
                In our case, the 'Dense' reward function will be defined as the negative Euclidean distance between 
                the achieved and the desired goal.

                If the environment contains a collision object, the reward will be extended by a penalty 
                defined by the distance from the object.

            The "Sparse" method can also be used:

                return -(self.__Euclidean_Norm(p - p_1) > self.__distance_threshold).astype(np.float32)

                Note:
                    The environment will return a reward if the task is completed.

        Args:
            (1) p [Vector<float> 1x3]: The point 'p(x, y, z)' represents the achieved goal.
            (2) p_1 [Vector<float> 1x3]: The point 'p_{1}(x, y, z)' represents the desired goal.

        Returns:
            (1) parameter [Vector<float> 1x1]: Calculated reward using the 'Dense' method.
        """
        
        if self.__mode == 'Default':
            return -self.__Euclidean_Norm(p - p_1).astype(np.float32)
        else:
            collision_obj_penalty = 1.0 / (1.0 + self.__Euclidean_Norm(p - self.__Env_Structure.Collision_Object.T.p.all().copy().astype(np.float32)))
            return -(self.__Euclidean_Norm(p - p_1) + collision_obj_penalty * self.__distance_threshold).astype(np.float32)  
 
    def is_success(self, p: tp.List[float], p_1: tp.List[float]) -> tp.List[float]:
        """
        Description:
            A function to calculate whether the desired target has been successfully found.

        Args:
            (1) p [Vector<float> 1x3]: The point 'p(x, y, z)' represents the achieved goal.
            (2) p_1 [Vector<float> 1x3]: The point 'p_{1}(x, y, z)' represents the desired goal.

        Returns:
            (1) parameter [Vector<float> 1x1]: Information whether the desired target has been successfully found.
        """
                
        return np.array(self.__Euclidean_Norm(p - p_1) < self.__distance_threshold, dtype=bool)
    
    def step(self, action: gym.spaces.Box) -> tp.Tuple[gym.spaces.Dict, float, bool, bool, tp.Dict]:
        """
        Description:
            A function to perform the action within the pre-defined environment and obtain the new observation space.

        Args:
            (1) action [gym.spaces.Box]: An action performed by the agent.

        Returns:
            (1) parameter [gym.spaces.Dict]: Observation of the current state.
            (2) parameter [float]: The amount of reward returned as a result of the execution of the action.
            (3) parameter [bool]: Information whether the terminal state (as defined under the MDP tasks) is reached. 
            (4) parameter [bool]: Information on whether a truncation condition outside the scope 
                                  of the MDP (Markov Decision Process) is satisfied.
            (5) parameter [Dictionary {'is_success': float}]: Information whether the desired target 
                                                              has been successfully found.
        """
                
        action = action.copy()
        action = np.clip(action, self.action_space.low, self.action_space.high)
    
        # Obtain a new position based on an input action to be performed within the environment.
        p_tmp = self.__PyBullet_Robot_Cls.T_EE.p.all().copy().astype(np.float32) + action * self.__action_step_factor
        
        # Transformation of point position in X, Y, Z axes.
        self.__P_EE.Transformation(p_tmp)

        # Determine if a given point is inside a search area.
        if self.__AABB_C_search.Is_Point_Inside(self.__P_EE) == True:
            # Obtain the inverse kinematics (IK) solution of the robotic structure from the desired TCP (tool center point).
            (successful, theta) = self.__PyBullet_Robot_Cls.Get_Inverse_Kinematics_Solution(HTM_Cls(None, np.float32).Rotation(self.__q_0, 'QUATERNION').Translation(p_tmp), 
                                                                                            self.__ik_properties, False)
            
            # Set the absolute position of the robot joints.
            self.__PyBullet_Robot_Cls.Set_Absolute_Joint_Position(theta, {'force': 100.0, 't_0': 0.0, 't_1': 0.1})

            truncated = not successful
        else:
            truncated = True

        # Get the homogeneous transformation matrix of the robot end-effector.
        self.__p = self.__PyBullet_Robot_Cls.T_EE.p.all().copy().astype(np.float32)

        # Get an observation of the current state.
        observation = np.concatenate([self.__p, 
                                      self.__PyBullet_Robot_Cls.T_EE_v[0:3]], dtype=np.float32)
        
    
        # Obtain additional output information from the execution of the action.
        terminated = bool(self.is_success(self.__p, self.__p_1))
        info = {'is_success': terminated}
        reward = float(self.compute_reward(self.__p, self.__p_1, info))

        return ({'observation': observation,
                 'achieved_goal': self.__p,
                 'desired_goal': self.__p_1.astype(np.float32)}, 
                reward,
                terminated,
                truncated,
                info)
    
    def reset(self, seed: tp.Optional[int] = None, options: tp.Optional[tp.Dict] = None) -> tp.Tuple[gym.spaces.Dict, tp.Dict]:
        """
        Description:
            A function to reset the pre-defined environment of the gym.

        Args:
            (1) seed [int]: A seed that is used to initialize the environment.
            (2) options [Dictionary {..}]: Additional information to determine how to reset the environment.

        Returns:
            (1) parameter [gym.spaces.Dict]: Observation of the initial state.
            (2) parameter [Dictionary {'is_success': float}]: Information whether the desired target 
                                                              has been successfully found.
        """
                
        super().reset(seed=seed, options=options)
        self.np_random, seed = gym.utils.seeding.np_random(seed)

        if self.__T is not None:
            self.__p_1 = self.__p_T
        else:
            # Obtain a random position within a defined configuration space.
            self.__p_1 = self.np_random.uniform(self.__min_vec3, self.__max_vec3).astype(np.float32)

        # Reset the absolute position of the robotic structure to 'Home'.
        self.__PyBullet_Robot_Cls.Reset(mode='Home', enable_ghost=False)

        # Obtain the homogeneous transformation matrix of the generated random position.
        T_rand = HTM_Cls(None, np.float32).Rotation(self.__q_0, 'QUATERNION').Translation(self.__p_1)
        self.__PyBullet_Robot_Cls.Transformation_External_Object('T_EE_Rand_Viewpoint', T_rand, False)
        self.__PyBullet_Robot_Cls.Transformation_External_Object('T_EE_Rand_Sphere', T_rand, False)

        # Get the homogeneous transformation matrix of the robot end-effector.
        self.__p = self.__PyBullet_Robot_Cls.T_EE.p.all().copy().astype(np.float32)

        # Get an observation of the initial state.
        observation = np.concatenate([self.__p, 
                                      self.__PyBullet_Robot_Cls.T_EE_v[0:3]], dtype=np.float32)

        return ({'observation': observation,
                 'achieved_goal': self.__p,
                 'desired_goal': self.__p_1}, 
                {'is_success': self.is_success(self.__p_0, self.__p_1)})

    def close(self) -> None:
        """
        Description:
            Disconnect the created environment from a physical server.
        """

        self.__PyBullet_Robot_Cls.Disconnect()