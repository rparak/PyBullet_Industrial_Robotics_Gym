# I4C_Gym

Optimize number of points!

https://robotics.farama.org/envs/fetch/reach/

plot, train (+ HER), add static point ...

```
Create:
conda create -n env_robotics_gym python=3.9

Install:
conda install -c conda-forge matplotlib
conda install -c conda-forge scienceplots
conda install -c conda-forge pandas
conda install pytorch::pytorch torchvision torchaudio -c pytorch
conda install -c conda-forge pybullet
conda install -c conda-forge gymnasium
conda install -c conda-forge stable-baselines3
```

**Comparison of training results**

| Type  | First successful result in a timestep | Percentage of success with a defined minimum success rate | mean(rollout/ep_rew_mean) | mean(rollout/ep_len_mean) | min(train/critic_loss) | min(train/actor_loss) |
| :---: | :---:                                 | :---:                                                     | :---:                     | :---:                     | :---:                  | :---:                 |
| DDPG  | 10484                                 |                                                           |                           |                           |                        |                       | 
| DDPG + HER |                                  |                                                           |                           |                           |                        |                       |
| SAC | Data | 10484                                 |                                                           |                           |                           |                        |                       |
| SAC + HER |                                   |                                                           |                           |                           |                        |                       |
| TD3 | Data | 10484                                 |                                                           |                           |                           |                        |                       |
| TD3 + HER |                                   |                                                           |                           |                           |                        |                       |



