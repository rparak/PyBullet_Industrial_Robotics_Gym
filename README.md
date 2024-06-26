# PyBullet Industrial Robotics Gym

<p align="center">
  <img src=https://github.com/rparak/PyBullet_Industrial_Robotics_Gym/blob/main/images/I4C.png width="800" height="450">
</p>

## Requirements

**Programming Language**

```bash
Python
```

**Import Libraries**
```bash
For additional information, please refer to the individual scripts (.py) or
the 'Installation Dependencies' section located at the bottom of the Readme.md file.
```

**Supported on the following operating systems**
```bash
Linux, macOS
```

## Project Description

The project focuses on motion planning for a wide range of robotic structures using deep reinforcement learning (DRL) algorithms to solve the problem of reaching a static or random target within a predefined configuration space. Its addresses the challenge of motion planning in environments under a variety of conditions, including environments with and without the presence of collision objects. It highlights the versatility and potential for future expansion through the integration of OpenAI Gym and PyBullet physics-based simulator.

<p align="center">
  <img src=https://github.com/rparak/PyBullet_Industrial_Robotics_Gym/blob/main/images/DRL_Env_UR3.png width="800" height="450">
</p>

To demonstrate the versatility of the proposed method in this project, robotic structures that are part of a robotic laboratory Industry 4.0 Cell were used. The laboratory contains a variety of robotic structures, which represents a set of the most common geometric representations used for experiments in robotic research. Namely, an industrial robot ABB IRB 120 with six degrees of freedom, the same robot extended by a linear axis providing seven DoF, a SCARA robot Epson LS3-B401S with four DoF, a dual-arm collaborative robot ABB IRB14000 with seven DoF on each arm, and finally, a collaborative robot Universal Robots UR3 with six DoF.

The solved problem was divided into two parts, with both parts focusing on reaching the target in a pre-defined configuration space. The first part, defined by the environment E1 , was focused on reaching the target within the configuration space without any external collision. The second part, defined by the environment E2, focused on the same problem, but with the presence of a statically positioned external collision object.

**Environment E1**

<p align="center">
  <img src=https://github.com/rparak/PyBullet_Industrial_Robotics_Gym/blob/main/images/PyBullet_Env_1.png width="800" height="450">
</p>

**Environment E2**

<p align="center">
  <img src=https://github.com/rparak/PyBullet_Industrial_Robotics_Gym/blob/main/images/PyBullet_Env_2.png width="800" height="450">
</p>

## Project Hierarchy

**../PyBullet_Industrial_Robotics_Gym/URDFs/**

The folder contains the Unified Robotics Description Format (URDF) structure for the specific robotic arms used in the experiment.

**../PyBullet_Industrial_Robotics_Gym/src/**

The folder contains the main parts (source code) of the project, which contains other dependencies. The individual sections contain additional information about the implementation of the solution, such as kinematics, collision detection, AI-gym, PyBullet simulation, etc.

```
$ ../src> ls
Industrial_Robotics_Gym PyBullet RoLE
```

Industrial_Robotics_Gym
- The main library contains the OpenAI Gym for training models using Deep Reinforcement Learning (DRL) algorithms.

PyBullet
- The PyBullet library, which contains functions and classes for the work with the physics simulation.

RoLE
- A robotics library that includes several function and classes for efficient control of robot structures.

**../PyBullet_Industrial_Robotics_Gym/Training/**

The folder contains the main scripts for training the DRL model using the selected algorithms, where the name of the scripts is the same as the name of the DRL method used for training.

Further information can be found in the scripts below.

```
$ ../Training> ls
train_ddpg.py train_sac.py train_td3.py
```

To train a model, simply run the specified script. It is necessary to set the parameters, including the type of robot structure and environment, as well as whether or not to use Hindsight Experience Replay (HER).

```
Deep Deterministic Policy Gradient (DDPG)
$ ../Training> python train_ddpg.py
Soft Actor Critic (SAC)
$ ../Training> python train_sac.py
Twin Delayed DDPG (TD3)
$ ../Training> python train_td3.py
```

**../PyBullet_Industrial_Robotics_Gym/Evaluation/**

The folder contains evaluation scripts for the proposed solution. The individual sections contain evaluations of the environment for training the model and evaluations of the trained model on specific scenarios.

```
$ ../Data> ls
Gym PyBullet
```

Gym
- Test scripts to evaluate the selected trained model for both scenarios and for a specific type of robotic structure. The folder includes the evaluation of the OpenAI Gym environment, the model's training and prediction evaluation for the specific robot structure in both scenarios, and the control of the robot using the trained model to reach the static target as well as the spline of the predicted path.

```
$ ../Evaluation/Gym> ls
Control Environment Model
```

PyBullet
- Test scripts to evaluate the environment for both scenarios. The folder also contains components for controlling the robot in the specific environment to evaluate the reachability of the configuration spaces.

```
$ ../PyBullet/Environment> python test_env.py
```

```
$ ../PyBullet/Control> python test_configuration_space_rand.py
$ ../PyBullet/Control> python test_configuration_space_vertices.py
```

**../PyBullet_Industrial_Robotics_Gym/Data/**

The folder contains data from the training progress, the trained model, and the prediction data obtained from a specific test.

```
$ ../Data> ls
Model Prediction Training
```

The experimental data are generated depending on the training or evaluation of a particular algorithm and robot structure.

## Installation Dependencies

It will be useful for the project to create a virtual environment using Conda. Conda is an open source package management system and environment management system that runs on Windows, macOS, and Linux. Conda quickly installs, runs and updates packages and their dependencies.

**Set up a new virtual environment called {name} with python {version}**
```
$ ../user_name> conda create -n {name} python={version}
$ ../user_name> conda activate {name}
```

**Installation of packages needed for the project**
```
Matplotlib
$ ../user_name> conda install -c conda-forge matplotlib

SciencePlots
$ ../user_name> conda install -c conda-forge scienceplots

PyBullet
$ ../user_name> conda install -c conda-forge pybullet

Pandas
$ ../user_name> conda install -c conda-forge pandas

SciPy
$ ../user_name> conda install -c conda-forge scipy

PyTorch, Torchvision, etc.
$ ../user_name> conda install pytorch::pytorch torchvision torchaudio -c pytorch
or 
$ ../user_name> conda install pytorch-nightly::pytorch torchvision torchaudio -c pytorch-nightly

Stable-Baselines3
$ ../user_name> conda install -c conda-forge stable-baselines3

Gymnasium
$ ../user_name> conda install -c conda-forge gymnasium
```

**Other useful commands for working with the Conda environment**
```
Deactivate environment.
$ ../user_name> conda deactivate

Remove environment.
$ ../user_name> conda remove --name {name} --all

To verify that the environment was removed, in your terminal window or an Anaconda Prompt, run.
$ ../user_name> conda info --envs

Rename the environment from the old name to the new one.
$ ../user_name> conda rename -n {old_name} {name_name}
```

## Video

In progress ...

## Contact Info:
Roman.Parak@outlook.com

## Citation (BibTex)
```

@Article{computation12060116,
AUTHOR = {Parák, Roman and Kůdela, Jakub and Matoušek, Radomil and Juříček, Martin},
TITLE = {Deep-Reinforcement-Learning-Based Motion Planning for a Wide Range of Robotic Structures},
JOURNAL = {Computation},
VOLUME = {12},
YEAR = {2024},
NUMBER = {6},
ARTICLE-NUMBER = {116},
URL = {https://www.mdpi.com/2079-3197/12/6/116},
ISSN = {2079-3197},
ABSTRACT = {The use of robot manipulators in engineering applications and scientific research has significantly increased in recent years. This can be attributed to the rise of technologies such as autonomous robotics and physics-based simulation, along with the utilization of artificial intelligence techniques. The use of these technologies may be limited due to a focus on a specific type of robotic manipulator and a particular solved task, which can hinder modularity and reproducibility in future expansions. This paper presents a method for planning motion across a wide range of robotic structures using deep reinforcement learning (DRL) algorithms to solve the problem of reaching a static or random target within a pre-defined configuration space. The paper addresses the challenge of motion planning in environments under a variety of conditions, including environments with and without the presence of collision objects. It highlights the versatility and potential for future expansion through the integration of OpenAI Gym and the PyBullet physics-based simulator.},
DOI = {10.3390/computation12060116}
}
```
