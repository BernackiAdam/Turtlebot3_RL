#! /usr/bin/env python3

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO, DQN
from TurtlebotEnv import Turtlebot_basic_RL
from Turtlebot_lidar_env import Turtlebot_lidar_RL
import rospy
from sensor_msgs.msg import LaserScan

env = Turtlebot_lidar_RL()
# check_env(env)

# # ewaluacja

ppo_path = "/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/1715470003_obs/1715470003_model/930000.zip"
model = DQN.load(ppo_path, env=env)
evaluate_policy(model, env, n_eval_episodes=100,render=True)


