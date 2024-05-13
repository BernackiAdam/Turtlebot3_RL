#! /usr/bin/env python3

from ast import Pass
import math

from sympy import true
# import turtlebot_util as tbu
import target
import time
import rospy
import robot_model

# zastosowane funkcje nagrody 

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



def based_on_action(angle_to_goal, distance_dif, action):
    reward = 0
    if angle_to_goal <= 0.30 and angle_to_goal >= -0.30:
        if action == 0:
            reward+= 5
        else:
            reward+=1
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

def distance_angle(distance_dif, curr_dis, entry_dis, angle):
    reward = 1
    angle_reward = 1
    distance = entry_dis- curr_dis
    if angle >= -1.3 and angle <= 1.3 and distance_dif >=0:
        angle_reward = 1
    else:
        angle_reward = 0
    if angle_reward != 0:
        reward = distance * angle_reward
    else:
        reward = abs(angle)*-1

    return reward

def based_on_obstacle(map):
    # if (map[0] or map[11]) and action == 0:
    #     reward = -1
    # elif (map[2] or map[3]) and action == 2:
    #     reward =-1
    # elif (map[8] or map[9]) and action == 1:
    #     reward =-1
    # else:
    #     reward =2
    map_len = len(map)
    close = False
    reward = 7
    for i in range(map_len):
        if i < int(map_len*0.0833) or i > int(map_len - map_len*0.0833):
            if map[i]<reward:
                reward = map[i]
    if reward < 0.3:
        close = True
    return reward,close
