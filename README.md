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

| Type  | successful timestep | Percentage | mean(rollout/ep_rew_mean) | mean(rollout/ep_len_mean) | min(train/critic_loss) | min(train/actor_loss) |
| :---: | :---:                                 | :---:                                                     | :---:                     | :---:                     | :---:                  | :---:                 |
| DDPG  | 10484 | 0.95862 | -0.38749 | 5.2994 | 7.5175e-07 | 0.077453 | 
| DDPG + HER | 5700 | 0.8324 | -0.38643 | 5.4301 | 1.1162e-06 | 0.063439 | 
| SAC   | 10588 | 0.96461 | -0.40878 | 5.6766 | 7.003e-06 | 0.014605 | 
| SAC + HER | 12035 | 0.9473 | -0.40717 | 5.6991 | 2.4319e-05 | 0.0089181 | 
| TD3   | 9454 | 0.9656 | -0.38657 | 5.2959 | 0.00017423 | 0.12341 | 
| TD3 + HER | 35717 | 0.78521 | -0.39556 | 5.7191 | 0.00031965 | 0.04994 |

