#! /usr/bin/env python3

import gymnasium as gym
from stable_baselines3 import PPO, DQN

import rospy
import os
import time

import tensorboard
import TurtlebotEnv
from Turtlebot_lidar_env import Turtlebot_lidar_RL
from std_srvs.srv import Empty

# model wytrenowany w pustym świecie w taki sposob aby dojezdzal do punktu
model_dir = "/home/bernacki/catkin_ws/src/turtlebot3_rl/Models_done/40k_empty_angle_distance/1715108529_model/40000.zip"
# model_dir = "/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/DQN_V2/1713634823/148000.zip"

models_dir = f"/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/{int(time.time())}_obs/{int(time.time())}_model"
log_dir = f"/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/{int(time.time())}_obs/{int(time.time())}_log"
if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(log_dir):
    os.makedirs(log_dir)
env = Turtlebot_lidar_RL()
env.reset()

model = DQN.load(model_dir, env=env, tensorboard_log = log_dir)
# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_dir)
pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
TIMESTEPS = 10000
for i in range(1,10000):
    unpause()
    model.learn(total_timesteps=TIMESTEPS, reset_num_timesteps=False,
                tb_log_name="DQN")
    model.save(f"{models_dir}/{TIMESTEPS*i}")
    # unpause()
    
 