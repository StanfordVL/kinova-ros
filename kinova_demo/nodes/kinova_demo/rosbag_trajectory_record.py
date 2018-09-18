#! /usr/bin/env python
"""Replay a trajectory in joint space recorded in a rosbag"""
import rospy
import actionlib

import roslib; roslib.load_manifest('kinova_demo')

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
from kinova_msgs.srv import *
from sensor_msgs.msg import JointState
import argparse

import importlib

import argparse
import sys

from std_srvs.srv import Empty

from copy import copy

import roslib; roslib.load_manifest('kinova_demo')

import actionlib

import kinova_msgs.msg
import geometry_msgs.msg
import tf
import std_msgs.msg
import math
import thread
from kinova_msgs.srv import *
import argparse
from robot_control_modules import *

import rospy
import rosbag

import actionlib

import numpy as np

import rospkg

initial_pose = [258.734375, 79.6875, 136.828125, 88.7045516968, 5.86363649368, 175.227279663, 0.0]

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']

recording = False
bag = None

def record_callback(msg):
    global recording
    global bag
    
    if recording:
        bag.write('/joint_states', msg)
        
        
def main():
    try:
        parser = argparse.ArgumentParser()
        parser.add_argument('filename')
        args = parser.parse_args()
        
        rospy.init_node('traj_record')     
        nb = raw_input('Setting right gravity vector, press return')
        service_address = '/m1n6s200_driver/in/change_gravity'	
        rospy.wait_for_service(service_address)	
        try:
            changeGravity = rospy.ServiceProxy(service_address, Empty)
            changeGravity()           
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
            
        print("Changed gravity")

        nb = raw_input('Starting gravity compensation, press return')
        #use service to set torque control parameters
        service_address = '/m1n6s200_driver/in/set_torque_control_parameters'
        rospy.wait_for_service(service_address)
        try:
            setTorqueParameters = rospy.ServiceProxy(service_address, SetTorqueControlParameters)
            setTorqueParameters()
        except rospy.ServiceException as e:
            print "Service call failed: %s"%e
      
        #use service to switch to torque control
        service_address = '/m1n6s200_driver/in/set_torque_control_mode'	
        rospy.wait_for_service(service_address)
        try:
            switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
            switchTorquemode(1)
        except rospy.ServiceException as e:
            print "Activating torque control mode call failed: %s"%e
            
        topic_name = '/m1n6s200_driver/in/joint_torque'
            
        pub = rospy.Publisher(topic_name, kinova_msgs.msg.JointTorque, queue_size=1)
        jointCmd = kinova_msgs.msg.JointTorque()
        jointCmd.joint1 = 0;
        jointCmd.joint2 = 0
        jointCmd.joint3 = 0
        jointCmd.joint4 = 0
        jointCmd.joint5 = 0
        jointCmd.joint6 = 0
        jointCmd.joint7 = 0
        
        pub.publish(jointCmd)
        rospy.sleep(0.1)
        pub.publish(jointCmd)
        rospy.sleep(0.1)
        pub.publish(jointCmd)
        rospy.sleep(0.1)
        pub.publish(jointCmd)
        rospy.sleep(0.1)
            
        global bag
        bag = rosbag.Bag(args.filename, 'w')
        
        rospy.Subscriber("/joint_states", JointState, record_callback)
        
        nb = raw_input('Start rosbag record with return')
        
        global recording
        recording = True  
        
        nb = raw_input('Stoping rosbag, press return')   
        
        recording = False  
        
        bag.close()

        nb = raw_input('Stoping gravity compensation, press return')
        #use service to switch to torque control
        service_address = '/m1n6s200_driver/in/set_torque_control_mode'	
        rospy.wait_for_service(service_address)
        try:
            switchTorquemode = rospy.ServiceProxy(service_address, SetTorqueControlMode)
            switchTorquemode(0)
        except rospy.ServiceException as e:
            print "Deactivating torque control mode call failed: %s"%e
        print("Done!")
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

if __name__ == '__main__':
    main()
