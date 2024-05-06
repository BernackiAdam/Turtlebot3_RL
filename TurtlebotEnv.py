#! /usr/bin/env python3

from queue import Empty
import gymnasium as gym
import numpy as np
from gymnasium import spaces

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
import time
from gazebo_msgs.msg import ModelState, ModelStates


import target
import robot_model
import reward_fcn





################################################################
# Pobranie danych bezpośrednio z wiadomości trurtlebot
################################################################
orientation_list = [0,0]
position_list = [0,0]
target_list = [0,0]
def odom_callback(msg):
    global orientation_list
    global position_list
    orientation_q = msg.pose.pose.orientation
    position = msg.pose.pose.position
    position_list = [position.x, position.y]
    orientation_list = [orientation_q.x, orientation_q.y, 
                        orientation_q.z, orientation_q.w]
def target_callback(msg):
    global target_list
    try:
        model_index = msg.name.index("target")
        pose = msg.pose[model_index]
        target_list[0] = pose.position.x
        target_list[1] = pose.position.y
    except ValueError:
        pass
rospy.init_node('turtle_movement')

rospy.Subscriber('/odom', Odometry, odom_callback)
rospy.Subscriber('gazebo/model_states', ModelStates, target_callback)

        ################################################################
        # klasa środowiska uczenia maszynowego
        ################################################################

class Turtlebot_basic_RL(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()

        ################################################################
        # określenie rodzajów i wymiarów przestrzeni akcji i obserwacji
        ################################################################


        # przestrzen obserwacji składa się z 5 elementów bedacych 
        # pozycjami robota i celu oraz 72 elementów będących wskazaniami
        # czujnika laserowego


        self.action_space = spaces.Discrete(3)
        self.observation_space = spaces.Box(low=-5, high=5,
                                            shape=(5+72,), dtype=np.float32)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.reward = 0
        self.episode = 0
        self.colision = False
        self.laser_data=[]
        for i in range(72):
            self.laser_data.append(6)
        
    def step(self, action):
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except:
            print("Unpause physics failed")

        xTarget = target_list[0]
        yTarget = target_list[1]

        self.episode += 1

        ################################################################        
        # Określenie akcji 
        ################################################################


        if action == 0:
            # Ruch do przodu
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)

        if action == 1:
            # Ruch w prawo 
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = -0.3
            self.cmd_vel_pub.publish(cmd_vel)

        if action == 2:
            # Ruch w lewo
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.2
            cmd_vel.angular.z = 0.3
            self.cmd_vel_pub.publish(cmd_vel)

        time.sleep(0.02)

        ################################################################
        # okreslenie przestrzeni obserwacji 
        ################################################################

        self.robot_x, self.robot_y = position_list
        _,_,self.yaw = euler_from_quaternion(orientation_list)
        
        self.robot_x = round(self.robot_x, 2)
        self.robot_y = round(self.robot_y, 2)
        xTarget = round(xTarget, 2)
        yTarget = round(yTarget, 2)
        self.yaw = round(self.yaw, 3)

        # self.laser_data,self.colision = robot_model.collect_laser_data()

        self.observation = [self.robot_x, self.robot_y, self.yaw,
                            xTarget, yTarget] + list(self.laser_data)
        
        self.observation = np.array(self.observation, dtype=np.float32)        

        self.angle_to_goal = robot_model.calculate_angle_to_goal(self.robot_x, self.robot_y, self.yaw,
                            xTarget, yTarget)
        self.distance_to_goal = reward_fcn.check_distance(self.robot_x, self.robot_y, self.yaw,
                            xTarget, yTarget)

        ################################################################
        # Określenie nagrody, terminacji oraz odcięcia (truncation)
        ################################################################

        if self.new_best_distance > self.distance_to_goal:
            self.new_best_distance = self.distance_to_goal

        distance_diffrence = self.new_best_distance - self.distance_to_goal

        if self.new_best_angle > self.angle_to_goal:
            self.new_best_angle = self.angle_to_goal

        angle_diffrence = self.new_best_angle - self.angle_to_goal
    
        # self.reward = reward_fcn.based_on_dif(self.angle_to_goal, distance_diffrence)
        self.reward = reward_fcn.based_on_action(self.angle_to_goal, distance_diffrence, action)
        
        ##############################################################
        # Terminacja dla robota poruszającego się po world
        ##############################################################

        
        # self.reward = 10-(self.distance_to_goal**2)

        # if self.colision:
        #     self.truncated = True
        #     self.reward -= 100
        # elif self.distance_to_goal <0.3:
        #     self.terminated = True
        #     self.reward += 2000
        # else:
        #     self.terminated = False
        #     self.truncated = False
        
        ##############################################################
        # Terminacja dla robota poruszającego się po empty_world
        ##############################################################

        if self.distance_to_goal > 20 or distance_diffrence < -0.9:
            self.truncated = True
            self.reward -= 100
        elif self.distance_to_goal < 0.3:
            self.terminated = True
            self.reward += 1000
        else:
            self.terminated = False
            self.truncated = False
        
        ################################################################
        # Wypisanie danych oraz zwrócenie istotnych informacji dla modelu
        ################################################################

        if self.episode % 1 == 0:
            print("Episode: {},      Angle to goal: {}".format(self.episode,round(self.angle_to_goal,5)))
            print("Reward: {},       Angle diffrence: {}".format(round(self.reward,5), round(angle_diffrence, 5)))
            print("Distance: {},     Distance diffrence: {}".format(round(self.distance_to_goal,5), round(distance_diffrence,5)))
            print("Target x: {},     target y: {}".format(xTarget, yTarget))
            print("Action: {}".format(action))
            print("========================================================")

        info = {}
        return self.observation, self.reward, self.terminated, self.truncated, info

    def reset(self, seed=None, options=None):
        self.unpause()
        self.terminated = False
        self.truncated = False
        self.reward = 0
        
        self.laser_data, self.colision = robot_model.collect_laser_data()
        self.unpause()
        self.new_best_distance = 100

        # Spawn targetu i robota dla empty_world
        self.target_x, self.target_y = target.spawn_target() 
        self.robot_x, self.robot_y, self.yaw = robot_model.reset_turtlebot()

        # Spawn targetu i robota dla areny turtlebot
        # self.target_x, self.target_y = target.spawn_target_world() 
        # self.robot_x, self.robot_y, self.yaw = robot_model.reset_turtlebot_world()
        self.new_best_angle = 5

        # Reset turtlebot dla world

        
        self.observation = [self.robot_x, self.robot_y, self.yaw,
                            self.target_x, self.target_y] + list(self.laser_data)
        
        self.observation = np.array(self.observation, dtype=np.float32)
        
        info = {}

        return self.observation, info

