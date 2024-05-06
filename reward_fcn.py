#! /usr/bin/env python3

from ast import Pass
import math
# import turtlebot_util as tbu
import target
import time
import rospy
import robot_model


def check_distance(robot_x, robot_y, yaw, target_x, target_y):

    distance_to_goal = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)
    return distance_to_goal

def only_aim_target(robot_x, robot_y, yaw, target_x, target_y):
# W przypadku tej funkcji, robot czasami dostaje nagrodę na poziomie 
# okolo 7-9 w momencie gdy jest całkowicie skierowany w przeciwną stronę do celu
    reward = 0
    angle_to_goal = robot_model.calculate_angle_to_goal(robot_x, robot_y, yaw, target_x, target_y)
    reward = ((4 - angle_to_goal)/2)**4
    return reward

def reward_wout_laser(robot_x, robot_y, yaw, target_x, target_y):

    reward = 0
    distance = check_distance(robot_x, robot_y, yaw, target_x, target_y)
    angle_to_goal = robot_model.calculate_angle_to_goal(robot_x, robot_y, yaw, target_x, target_y)
    
    reward = (10 - distance*2) + (3.14 - angle_to_goal)*5

    return reward/10

def new_target(robot_x, robot_y):
    target_x, target_y = target.spawn_target_world()
    entry_distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

    return entry_distance, target_x, target_y

def distance_angle(distance_dif, curr_dis, entry_dis, angle):
    reward = 0
    distance = entry_dis- curr_dis
    if angle >= -1.31 and angle <= 1.31 and distance_dif >=0:
        reward = 2**distance
    elif distance_dif <0:
        reward = distance_dif*3
    else:
        reward = -1


    return reward

def based_on_action(angle_to_goal, distance_dif, action):
    reward = 0
    if angle_to_goal <= 0.20 and angle_to_goal >= -0.20:
        if action == 0:
            reward+= 5
        else:
            reward-=2
    elif angle_to_goal > 0:
        if action == 2:
            reward += 3
        else:
            reward -=2
    else:
        if action == 1:
            reward += 3
        else:
            reward -=2

    return reward


def based_on_obstacle(map, action):
    if (map[0] or map[11]) and action == 0:
        reward = -1
    elif (map[2] or map[3]) and action == 2:
        reward =-1
    elif (map[8] or map[9]) and action == 1:
        reward =-1
    else:
        reward =2
    return reward
# stworzyc funkcje nagrody ktora nagradza robota za dojechanie do celu i po 350 epizodach zaczyna odejmowac punkty 