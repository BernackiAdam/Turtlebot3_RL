#! /usr/bin/env python3

from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO, DQN
from TurtlebotEnv import Turtlebot_basic_RL
import rospy
from sensor_msgs.msg import LaserScan
env = Turtlebot_basic_RL()
# check_env(env)

# # ewaluacja
ppo_path = "/home/bernacki/catkin_ws/src/turtlebot3_rl/Models/DQN-300k BEST/250000.zip"
model = DQN.load(ppo_path, env=env)
evaluate_policy(model, env, n_eval_episodes=100,render=True)


