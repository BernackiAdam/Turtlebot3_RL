#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import DeleteModel, SpawnModel, SetModelState
import random
import os
import time

model_states = []

def spawn_target():
    # rospy.init_node("set_pose")
    start_time = rospy.Time.now()
    while rospy.Time.now() - start_time < rospy.Duration(10):
            msg = rospy.wait_for_message("gazebo/model_states", ModelStates, timeout=10.0)
            if "target" in msg.name:
             
                rospy.wait_for_service('/gazebo/delete_model')
                delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                model_name = "target"
                delete_model_proxy(model_name)

                rospy.wait_for_service('/gazebo/spawn_sdf_model')
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                time.sleep(0.3)
                model_name = "target"  
                with open("/home/bernacki/catkin_ws/src/turtlebot3_rl/Scripts/target.sdf", "r") as f:
                    model_xml = f.read()
                spawn_pose = Pose()  
                spawn_pose.position.x = random.uniform(-5, 5)  
                spawn_pose.position.y = random.uniform(-5, 5)  
                spawn_pose.position.z = 0.1  
                spawn_model(model_name, model_xml, " ", spawn_pose, "world")
                return spawn_pose.position.x, spawn_pose.position.y
            else:
                rospy.wait_for_service('/gazebo/spawn_sdf_model')
                spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

                model_name = "target"  
                with open("/home/bernacki/catkin_ws/src/turtlebot3_rl/Scripts/target.sdf", "r") as f:
                    model_xml = f.read()
                spawn_pose = Pose()  
                spawn_pose.position.x = random.uniform(-5, 5)  
                spawn_pose.position.y = random.uniform(-5, 5)  
                spawn_pose.position.z = 0.1  
                spawn_model(model_name, model_xml, " ", spawn_pose, "world")
                return spawn_pose.position.x, spawn_pose.position.y

def spawn_target_world():
    # rospy.init_node("set_pose")
    # poses_x = [-2, -1.5, -1, -0.5, 0, 0.5, 1, 1.5, 2]
    # poses_y = [-2, -1.5, -1, 0, 0.5, 1, 1.5, 2]
    # forbidden_poses = [(-1, 1), (0, 1), (1, 1),
    #                     (-1, 0), (0, 0), (1, 0),
    #                     (-1, -1), (0, -1), (1, -1),
    #                     (-2,2), (2,2),
    #                     (-2,-2), (2,-2),
    #                     (-1.5, 2), (1.5, 2),
    #                     (-1.5, -2), (1.5, -2), (-2.0, -0.5)]
    # poses = [(2, 0.5), (3.8, 1), (3.8, -1.5), (1, -2),
    #          (2.5, -2.5), (0,-2)]
    poses = [(1, 1.5),(1, 0.5), (0, 1.7), (1.5, 0.5), (1.5 ,-0.5), (0.5, -1.5)]
    

    start_time = rospy.Time.now()
    position = list(random.choice(poses))
    # while True:
    #     position[0] = poses_x[random.randint(0,8)]
    #     position[1] = poses_y[random.randint(0,7)]
    #     if tuple(position) in forbidden_poses:
    #         continue
    #     else:
    #         break
    
    
    while rospy.Time.now() - start_time < rospy.Duration(10):
        msg = rospy.wait_for_message("gazebo/model_states", ModelStates, timeout=10.0)
        if "target" in msg.name:
            
            rospy.wait_for_service('/gazebo/delete_model')
            delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            model_name = "target"
            delete_model_proxy(model_name)

            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            time.sleep(0.3)
            model_name = "target"  
            with open("/home/bernacki/catkin_ws/src/turtlebot3_rl/Scripts/target.sdf", "r") as f:
                model_xml = f.read()
            spawn_pose = Pose()  
            spawn_pose.position.x = position[0] 
            spawn_pose.position.y = position[1]
            spawn_pose.position.z = 0.1  
            spawn_model(model_name, model_xml, " ", spawn_pose, "world")
            return spawn_pose.position.x, spawn_pose.position.y
        else:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

            model_name = "target"  
            with open("/home/bernacki/catkin_ws/src/turtlebot3_rl/Scripts/target.sdf", "r") as f:
                model_xml = f.read()
            spawn_pose = Pose()  
            spawn_pose.position.x = position[0] 
            spawn_pose.position.y = position[1]  
            spawn_pose.position.z = 0.1  
            spawn_model(model_name, model_xml, " ", spawn_pose, "world")
            return spawn_pose.position.x, spawn_pose.position.y




if __name__ == "__main__":

    spawn_target_world()