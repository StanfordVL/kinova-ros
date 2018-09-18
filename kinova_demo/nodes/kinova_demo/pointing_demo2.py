#! /usr/bin/env python
"""Replay a trajectory in joint space recorded in a rosbag"""

import rospy
import actionlib

import roslib; roslib.load_manifest('kinova_demo')

from rosbag_trajectory_replay import *

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse

from geometry_msgs.msg import Pose, Twist

from std_msgs.msg import Int16, UInt8
from jr2_msgs.msg import EmotionType

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

import importlib

import argparse
import sys

from copy import copy

import actionlib

from robot_control_modules import *

import rospy
import rosbag

import actionlib

import numpy as np

import rospkg

import tf2_ros

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

initial_pose_gaze = [270, 70, 70, 90, 0, 180, 0.0]

arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']


""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [0] # number of joints is defined in __main__

listener = None
tfBuffer = None

cmd_vel_pub = None

currentBaseOrientation = None

def getcurrentJointCommand():
    # wait to get current position
    topic_address = '/m1n6s200_driver/out/joint_command'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'Position listener obtained message for joint position. '
    
def setcurrentJointCommand(feedback):
    global currentJointCommand
    
    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])

    #print 'Current joint pose is: '
    #print currentJointCommand
    
def kinova_robotTypeParser(kinova_robotType_):
    """ Argument kinova_robotType """
    global robot_category, robot_category_version, wrist_type, arm_joint_number, robot_mode, finger_number, prefix, finger_maxDist, finger_maxTurn 
    robot_category = kinova_robotType_[0]
    robot_category_version = int(kinova_robotType_[1])
    wrist_type = kinova_robotType_[2]
    arm_joint_number = int(kinova_robotType_[3])
    robot_mode = kinova_robotType_[4]
    finger_number = int(kinova_robotType_[5])
    prefix = kinova_robotType_ + "_"
    finger_maxDist = 18.9/2/1000  # max distance for one finger in meter
    finger_maxTurn = 6800  # max thread turn for one finger
        
def getCurrentBaseOrientation():
    # wait to get current position
    topic_address = '/segway/odometry/local_filtered'
    rospy.Subscriber(topic_address, Odometry, getCurrentBaseOrientationCB)
    rospy.wait_for_message(topic_address, Odometry)
    
def getCurrentBaseOrientationCB(base_orientation_msg):
    global currentBaseOrientation    
    currentBaseOrientation = base_orientation_msg.pose.pose.z
    print 'Current joint pose is: ' + str(currentBaseOrientation)
    
def pointing_cb(point_msg):
    
    x_axis = np.array([1,0,0])
    y_axis = np.array([0,1,0])
    z_axis = np.array([0,0,1])
    
    global listener
    global tfBuffer
    global cmd_vel_pub
    
    #Tranform point from base link to robot 
    #trans = tfBuffer.lookup_transform('base_link', 'm1n6s200_link_3', rospy.Time())
    #trans2 = tfBuffer.lookup_transform('base_link', 'm1n6s200_link_5', rospy.Time())
    
    trans_point1 = None
    trans_point2 = None
    
    point_msg_cpy = copy(point_msg)
    
    if point_msg_cpy.point.z == 0:
        point_msg_cpy.point.z = 1.8
        
    print "here"
        
    listener.waitForTransform("/base_link", "/m1n6s200_link_3", rospy.Time(), rospy.Duration(10.0))
    listener.waitForTransform("/base_link", "/m1n6s200_link_5", rospy.Time(), rospy.Duration(10.0))
    
    try:
        trans_point1 = listener.transformPoint('/m1n6s200_link_3',point_msg_cpy )
        trans_point2 = listener.transformPoint('/m1n6s200_link_5',point_msg_cpy )
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print 'No tranform'
        return False
    
    point_axis1 = np.array([trans_point1.point.x,trans_point1.point.y,0])
    point_axis_norm1 = point_axis1/np.linalg.norm(point_axis1)
    
    angle_to_y1 = np.arccos(np.dot(y_axis, point_axis_norm1))
    cross_to_y1 = np.cross(y_axis, point_axis_norm1)
    if np.dot(z_axis, cross_to_y1) < 0:
        angle_to_y1 = -angle_to_y1
    
    point_axis2 = np.array([trans_point2.point.x,trans_point2.point.y,0])
    point_axis_norm2 = point_axis2/np.linalg.norm(point_axis2)
    angle_to_y2 = np.arccos(np.dot(-y_axis, point_axis_norm2))
    cross_to_y2 = np.cross(-y_axis, point_axis_norm2)
    if np.dot(z_axis, cross_to_y2) < 0:
        angle_to_y2 = -angle_to_y2
    
    #print point_msg
    #print trans_point1
    #print point_axis1
    #print point_axis_norm1
    #print trans_point2
    
    #print "angle_to_y1 " + str(angle_to_y1)
    #print "angle to y2 " + str(angle_to_y2)
    
    getcurrentJointCommand()
    
    new_goal = copy(currentJointCommand)
    new_goal[2] += np.rad2deg(angle_to_y1)
    new_goal[4] += np.rad2deg(angle_to_y2)
    print new_goal
    
    result = joint_position_client(new_goal, 'm1n6s200_')
    
    rospy.sleep(3)
    
    #cmd_vel = Twist()
    
    #cmd_vel.linear.x = 0
    #gain_rot = 0.5    

    ## Send the motion command to the base
    #getCurrentBaseOrientation()
    #global currentBaseOrientation    
    #if currentBaseOrientation is None:
        #print "Current orientation of the base was not retrieved"
        #exit(-1)
    #initial_base_angle = currentBaseOrientation
    
    #goal_base_angle = initial_base_angle + angle_to_x

    #while(current_angle < goal_base_angle):
        #getCurrentBaseOrientation()
        #angle_to_x = goal_base_angle - currentBaseOrientation
        #cmd_vel.angular.z = np.min(0.1, angle_to_x * gain_rot)
        #cmd_vel_pub.publish(cmd_vel)        
        #rospy.sleep(0.1)   
       

def main():
    """Joint Trajectory Example: Simple Action Client
    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.
    Make sure to start the joint_trajectory_action_server
    first. Then run this example command a short series of trajectory points for the arm
    to follow.
    """
    global currentJointCommand
    global initial_pose_gaze
    kinova_robotTypeParser('m1n6s200')
    arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
    finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']
    
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client")
    print("Running. Ctrl-c to quit")
    
    result = joint_position_client(initial_pose_gaze, 'm1n6s200_')  
    
    global listener
    listener = tf.TransformListener()
    
    rospy.Subscriber('/gaze_point_3d', PointStamped, pointing_cb, queue_size=1)
    
    global cmd_vel_pub
    
    cmd_vel_pub = rospy.Publisher("segway/twist/cmd_vel", Twist, queue_size=5)
    
    print 'Pointing arm'

    rospy.spin()

    print("Exiting - Joint Trajectory Action Complete")

if __name__ == "__main__":
    
    currentJointCommand = [0]*7
    main()
        
