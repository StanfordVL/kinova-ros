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

from std_msgs.msg import Int16, UInt8
from jr2_msgs.msg import EmotionType

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

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']


""" Global variable """
arm_joint_number = 0
finger_number = 0
prefix = 'NO_ROBOT_TYPE_DEFINED_'
finger_maxDist = 18.9/2/1000  # max distance for one finger
finger_maxTurn = 6800  # max thread rotation for one finger
currentJointCommand = [0] # number of joints is defined in __main__

def getcurrentJointCommand():
    # wait to get current position
    topic_address = '/m1n6s200_driver/out/joint_command'
    print 'mm'
    rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, setcurrentJointCommand)
    print 'pp'
    rospy.wait_for_message(topic_address, kinova_msgs.msg.JointAngles)
    print 'Position listener obtained message for joint position. '
    
def setcurrentJointCommand(feedback):
    global currentJointCommand
    
    print 'hola'

    currentJointCommand_str_list = str(feedback).split("\n")
    for index in range(0,len(currentJointCommand_str_list)):
        temp_str=currentJointCommand_str_list[index].split(": ")
        currentJointCommand[index] = float(temp_str[1])

    print 'Current joint pose is: '
    print currentJointCommand
    
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
    
initial_pose = [258.734375, 79.6875, 136.828125, 88.7045516968, 5.86363649368, 175.227279663, 0.0]
neutral = [[0, 0, 0, 0, 0, 0, 0]]
happy = [[0, -10, -20, 0, 0, 0, 0], [0, 10, 10, 0, 0, 0, 0], [0, -10, -20, 0, 0, 0, 0], [0, 10, 10, 0, 0, 0, 0]]
sad = [[20, 20, -50, -180, 0, 0, 0]]
hello = [[15, 0, 0, 0, 0, 0, 0], [-15, 0, 0, 0, 0, 0, 0], [15, 0, 0, 0, 0, 0, 0], [-15, 0, 0, 0, 0, 0, 0]]
emotions_to_gestures = {EmotionType.NEUTRAL:neutral, EmotionType.HAPPY:happy, EmotionType.HAPPY2:happy, EmotionType.SAD:sad, EmotionType.EMBARRASSED+1:hello}
emotions_to_gestures_str = {EmotionType.NEUTRAL:'neutral', EmotionType.HAPPY:'happy', EmotionType.HAPPY2:'happy2', EmotionType.SAD:'sad', EmotionType.EMBARRASSED+1:'hello'}
executing = False
executing_fingers = False
executing_arm_pose = 0

neutral_fingers = [[0, 0]]
happy_fingers = [[finger_maxTurn, finger_maxTurn], [0,0], [finger_maxTurn,finger_maxTurn], [0,0]]
sad_fingers = [[0, 0]]
hello_fingers = [[finger_maxTurn, finger_maxTurn], [0,0], [finger_maxTurn,finger_maxTurn], [0,0]]
emotions_to_finger_gestures = {EmotionType.NEUTRAL:neutral_fingers, EmotionType.HAPPY:happy_fingers, EmotionType.HAPPY2 :happy_fingers, EmotionType.SAD:sad_fingers, EmotionType.EMBARRASSED+1:hello_fingers}

neutral_bag = '~/emotion_bags/neutral.bag'
happy_bag = '~/emotion_bags/happy.bag'
sad_bag = '~/emotion_bags/sad.bag'
hello_bag = '~/emotion_bags/hello.bag'

emotions_to_gesture_bags = {EmotionType.NEUTRAL:neutral_bag, EmotionType.HAPPY:happy_bag, EmotionType.HAPPY2:happy_bag, EmotionType.SAD:sad_bag, EmotionType.EMBARRASSED+1:hello_bag}


def arm_gesture_cb(emotion_msg):
    global executing
    
    if emotion_msg.data in emotions_to_gestures.keys():
    
        print 'Emotion received'
        if not executing:
            executing = True
            execute_gesture(emotion_msg.data)   
            executing = False     
        
def execute_gesture(emotion_label):
    global initial_pose
    global executing
    global executing_arm_pose
    result = joint_position_client(initial_pose, 'm1n6s200_') 
    
    print 'Executing emotion ' + emotions_to_gestures_str[emotion_label]
    
    
    
    print emotions_to_gestures[emotion_label]
    
    executing_arm_pose = len(emotions_to_gestures[emotion_label]) - 1
    
    for i, pose in enumerate(emotions_to_gestures[emotion_label]):
        print 'pose ' + str(i)
        executing_arm_pose = len(emotions_to_gestures[emotion_label]) - 1 - i
        result = joint_position_client((np.array(initial_pose) + np.array(pose)).tolist(), 'm1n6s200_')


def execute_gesture_bag(emotion_label):
	global initial_pose
    global executing
    global executing_arm_pose
    
    print 'Executing emotion ' + emotions_to_gestures_str[emotion_label]
    print 'Executing bag ' + emotions_to_gestures_bag[emotion_label]

    replay_rosbag_trajectory(emotions_to_gestures_bag[emotion_label])   
        
        
def fingers_gesture_cb(emotion_msg):
    global executing_fingers
    
    if emotion_msg.data in emotions_to_gestures.keys():
        print 'Emotion received (Fingers)'
        if not executing_fingers:
            executing_fingers = True
            print 'inside'
            execute_gesture_fingers(emotion_msg.data)  
            executing_fingers = False   
        
def execute_gesture_fingers(emotion_label):
    global initial_pose
    global executing_fingers
    global executing_arm_pose
    
    while executing_arm_pose != 0:
        print 'fingers pose ' + str(executing_arm_pose)
        print emotions_to_finger_gestures[emotion_label][executing_arm_pose]
        result = gripper_client(emotions_to_finger_gestures[emotion_label][executing_arm_pose] + [0], 'm1n6s200_')
    print 'finished'
        

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
    global initial_pose
    kinova_robotTypeParser('m1n6s200')
    arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
    finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']
    
    
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client")
    print("Running. Ctrl-c to quit")
    
    
    #nb = raw_input('Moving to the starting position of the emotions, press return')   
    
    print initial_pose
    
    result = joint_position_client(initial_pose, 'm1n6s200_')
    
    #nb = raw_input('Listening to emotion commands, press return')   
    
    rospy.Subscriber('/emotion_type', UInt8, arm_gesture_cb)
    #rospy.Subscriber('/emotion_type2', UInt8, fingers_gesture_cb)
    #Fingers and arm cannot be controlled at the same time separately
    
    rospy.spin()

    print("Exiting - Joint Trajectory Action Complete")

if __name__ == "__main__":
    
    currentJointCommand = [0]*7
    main()
        
