#! /usr/bin/env python3

import gymnasium as gym
from stable_baselines3 import PPO, DQN, TD3

import rospy
import os
import time
import numpy as np

import tensorboard
import TurtlebotEnv
# from TurtlebotEnv import Turtlebot_basic_RL
from Turtlebot_lidar_env import Turtlebot_lidar_RL
from std_srvs.srv import Empty



models_dir = f"/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/{int(time.time())}"
log_dir = f"/home/bernacki/catkin_ws/src/turtlebot3_rl/Logs/{int(time.time())}"
if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)


env = Turtlebot_lidar_RL()
env.reset()

# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
model = DQN("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)

pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
TIMESTEPS = 10000
for i in range(1,10000):
    unpause()
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False,
                tb_log_name="DQN")
    model.save(f"{models_dir}/{TIMESTEPS*i}")
    # unpause()
obs, info = env.reset()

