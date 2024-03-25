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

The project focuses on the problem of motion planning for a wide range of robotic structures using deep reinforcement learning (DRL) algorithms to solve the problem of reaching a static or random target within a predefined configuration space.

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
$ ../Training> python train_ddpg.py
```

**../PyBullet_Industrial_Robotics_Gym/Evaluation/**

The folder contains evaluation scripts for the proposed solution. The individual sections contain evaluations of the environment for training the model and evaluations of the trained model on specific scenarios.

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

....

## Contact Info:
Roman.Parak@outlook.com

## Citation (BibTex)
```
....
```
