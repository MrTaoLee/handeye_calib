#!/usr/bin/env python
import sys
import os
import math
import rospy
import copy
import numpy as np
import tf
import moveit_commander
import motion_primitives
import yaml
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('handeye_calibration', anonymous=True)  

moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander() 
scene = moveit_commander.PlanningSceneInterface() 
group = moveit_commander.MoveGroupCommander("manipulator") 
path = '/home/john/catkin_ws/src/handeye_calib/input/'
config_path = "/home/john/catkin_ws/src/handeye_calib/launch/robot_config.yaml"

def get_save_image(count):
    bridge = CvBridge()
    rospy.loginfo("Getting image...")
    image_msg = rospy.wait_for_message("/image", Image)
    cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    cv2.imwrite(os.path.join(path, 'images/camera0/image'+str(count)+'.png'),cv_image)
    rospy.loginfo("Got image and saved!")

def get_write_pose(count):
    p = group.get_current_pose().pose
    trans_tool0 = [p.position.x, p.position.y, p.position.z]
    rot_tool0 = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
    T = tf.TransformerROS().fromTranslationRotation(trans_tool0, rot_tool0)
    
    robot_pose_file = open(os.path.join(path, 'robot_cali.txt'),"a+") 
    for i in range(4):
        robot_pose_file.write(str(T[i][0]) + " " + str(T[i][1]) + " " + str(T[i][2]) + " " + str(T[i][3]) + " " + "\n") 
    robot_pose_file.write("\n") 
    robot_pose_file.close() 

def clean_log():
    robot_pose_file = open(os.path.join(path, 'robot_cali.txt'),"w+") 
    robot_pose_file.close() 

def calibration(trans_mag, ori_mag): 
    p = group.get_current_pose().pose
    pos = [p.position.x, p.position.y, p.position.z]
    ori = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w] 
    T_wg = tf.TransformerROS().fromTranslationRotation(pos, ori)
       
    pos = []
    rot = []
    pos.append(copy.deepcopy([0,  trans_mag,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(-ori_mag, 0, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,  trans_mag,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0,  trans_mag,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(-ori_mag, ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,  trans_mag,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0,    0,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,    0,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0, -trans_mag,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(ori_mag, ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0, -trans_mag,  trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0, -trans_mag,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(ori_mag, 0, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0, -trans_mag,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0, -trans_mag, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(ori_mag, -ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0, -trans_mag, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0,    0, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, -ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,    0, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0,  trans_mag, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(-ori_mag, -ori_mag, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,  trans_mag, -trans_mag]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    pos.append(copy.deepcopy([0,    0,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    pos.append(copy.deepcopy([0,    0,    0]))
    rot.append(copy.deepcopy( tf.transformations.quaternion_from_euler(0, 0, 0, 'rzyx') ))
    
    robot_pose_file = open(os.path.join(path, 'robot_cali.txt'),"a+") 
    robot_pose_file.write(str(len(pos))+"\n") 
    robot_pose_file.close() 

    for i in range(len(pos)): 
        T_gg_target = tf.TransformerROS().fromTranslationRotation(pos[i], rot[i]) 
        T_wg_target = np.matmul(T_wg, T_gg_target)
        trans_target = T_wg_target[:3,3]
        rot_target = tf.transformations.quaternion_from_matrix(T_wg_target[:4][:4])
        pose_target = np.concatenate((trans_target, rot_target), axis=None)
        motion_primitives.set_pose(pose_target)
        get_save_image(i)
        get_write_pose(i)

if __name__ == '__main__':
    with open(config_path, 'r') as stream:
        try:
            config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    try:
        tcp_speed = config['tcp_speed']
        motion_range = config['motion_range']
        orientation_range = config['orientation_range']
        group.set_max_velocity_scaling_factor(tcp_speed)
        clean_log()
        calibration(motion_range, orientation_range)
    
    except rospy.ROSInterruptException: pass
