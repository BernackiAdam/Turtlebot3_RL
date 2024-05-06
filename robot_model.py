#! /usr/bin/env python3

from numpy import Inf
import numpy as np
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from sensor_msgs.msg import LaserScan

import time
import random
import os
import math
def reset_turtlebot():
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Tworzenie nowego obiektu typu ModelState dla TurtleBota
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'  # Nazwa modelu TurtleBota w symulacji Gazebo

        # Ustawianie losowej pozycji
        state_msg.pose.position.x  =random.uniform(-2, 2)  # Zakres współrzędnej x
        state_msg.pose.position.y  =random.uniform(-2, 2)  # Zakres współrzędnej y
        state_msg.pose.position.z = 0.0  # Ustawienie wysokości na płaszczyźnie

        # Resetowanie orientacji
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = random.uniform(-1,1)
        state_msg.pose.orientation.w = 1.0

        orientations = [state_msg.pose.orientation.x,
                     state_msg.pose.orientation.y,
                     state_msg.pose.orientation.z,
                     state_msg.pose.orientation.w]
        

        _,_,yaw = euler_from_quaternion(orientations)
        # print(yaw)

        # Wysłanie żądania ustawienia nowego stanu
        resp = set_state(state_msg)

        # rospy.loginfo("TurtleBot został zresetowany i umieszczony w losowym miejscu na mapie.")
        return state_msg.pose.position.x , state_msg.pose.position.y , yaw
        
    except rospy.ServiceException as e:
        rospy.logerr("Błąd podczas ustawiania stanu modelu: %s" % e)

def reset_turtlebot_world():
    rospy.wait_for_service('/gazebo/set_model_state')

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Tworzenie nowego obiektu typu ModelState dla TurtleBota
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'  # Nazwa modelu TurtleBota w symulacji Gazebo

        # Ustawianie losowej pozycji
        # state_msg.pose.position.x  =-2.0  # Zakres współrzędnej x
        # state_msg.pose.position.y  =-0.5 # Zakres współrzędnej y
        state_msg.pose.position.x  =0.0  # Zakres współrzędnej x
        state_msg.pose.position.y  =0.0 # Zakres współrzędnej y
        state_msg.pose.position.z = 0.0  # Ustawienie wysokości na płaszczyźnie

        # Resetowanie orientacji
        state_msg.pose.orientation.x = 0.0
        state_msg.pose.orientation.y = 0.0
        state_msg.pose.orientation.z = 0.0
        state_msg.pose.orientation.w = 1.0

        orientations = [state_msg.pose.orientation.x,
                     state_msg.pose.orientation.y,
                     state_msg.pose.orientation.z,
                     state_msg.pose.orientation.w]
        

        _,_,yaw = euler_from_quaternion(orientations)
        # print(yaw)

        # Wysłanie żądania ustawienia nowego stanu
        resp = set_state(state_msg)

        # rospy.loginfo("TurtleBot został zresetowany i umieszczony w losowym miejscu na mapie.")
        return state_msg.pose.position.x , state_msg.pose.position.y , yaw
        
    except rospy.ServiceException as e:
        rospy.logerr("Błąd podczas ustawiania stanu modelu: %s" % e)

def calculate_yaw(orientation_list):
    _, _, current_yaw = euler_from_quaternion(orientation_list)
    return current_yaw

def calculate_angle_to_goal(robot_x, robot_y, yaw, target_x, target_y):

    goal_angle = math.atan2(target_y - robot_y, target_x - robot_x)
    #normalize angle
    angle = goal_angle-yaw
    angle_to_goal = math.atan2(math.sin(angle), math.cos(angle))
    # angle_to_goal = abs(angle_to_goal)
    return angle_to_goal

   # mniejsze od zera - wtedy nagroda za skret w prawo
   # wieksze od zera - nagroda za skret w lewo

# https://github.com/erlerobot/gym-gazebo/blob/master/gym_gazebo/envs/turtlebot/gazebo_circuit2_turtlebot_lidar.py

def collect_laser_data():
    
    data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
    min_range = 0.17
    close_range = 0.35
    laser_data = np.zeros(len(data.ranges)//5)
    colision = False
    close = False
    for i in range(0, len(data.ranges), 5):
        index = i//5
        # if data.ranges[i] == float('Inf') or np.isinf(data.ranges[i]):
        #     laser_data[index] = 6
        # elif np.isnan(data.ranges[i]):
        #     laser_data[index] = 0
        # else:
        #     laser_data[index]=data.ranges[i]
        laser_data[index]=data.ranges[i]
        if (close_range > data.ranges[i] > 0.01):
            close = True

        if (min_range > data.ranges[i] > 0.01):
            colision = True
            

    return laser_data, colision, close

def compress_laser_data(lidar_data):

    min_range = 0.17
    close_range = 0.27
    laser_data = np.zeros(len(lidar_data)//5)
    colision = False
    close = False
    for i in range(0, len(lidar_data), 5):
        index = i//5
        if lidar_data[i] == float('Inf') or np.isinf(lidar_data[i]):
            laser_data[index] = 6
        elif np.isnan(lidar_data[i]):
            laser_data[index] = 0
        else:
            laser_data[index]=round(lidar_data[i],3)

        if (close_range > lidar_data[i] > 0.01):
            close = True

        if (min_range > lidar_data[i] > 0.01):
            colision = True
       # zrobic model w ktorym w przestrzeni obserwacji beda 3 wartosci bool odnosnie      
        # czy blisko czy daleko 
        
    return laser_data, colision, close

def create_obstacle_map(data):
    # scan = rospy.wait_for_message('/scan', LaserScan, timeout=5)
    # data = scan.ranges
    obstacle_map = []
    index = 0
    for i in range(24):
        obstacle_map.append(6)
        index = i*(int(len(data)/24))
        for j in range(int(len(data)/24)):
            if data[j+index] <obstacle_map[i]:
                obstacle_map[i] = round(data[j+index], 3)
    # print(obstacle_map)
    return obstacle_map

def calculate_entry_distance(robot_x, robot_y, target_x, target_y):

    entry_distance = math.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

    return entry_distance

if __name__ == '__main__':
    rospy.init_node('reset_turtlebot_node')
    # reset_turtlebot()
    # start_time = time.time()
    # data = collect_laser_data()
    # stop_time = time.time()
    
    # for i in data:
    #     print(i)
    # print(stop_time-start_time)
    create_obstacle_map()
    
