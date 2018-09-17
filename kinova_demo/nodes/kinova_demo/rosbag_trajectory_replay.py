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

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']

class Trajectory(object):
    def __init__(self):
        ns = '/m1n6s200/'
        rospy.loginfo("Connecting to " + ns + "follow_joint_trajectory")
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        global arm_joints_names
        global finger_joints_names
        self._goal.trajectory.joint_names = arm_joints_names# + finger_joints_names
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

    def add_point(self, positions, velocities, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.velocities = copy(velocities)
        point.time_from_start = time
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=timeout)

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']


def replay_rosbag_trajectory(filename):
    bag = rosbag.Bag(filename)
    arm_joints_names = ['m1n6s200_joint_1', 'm1n6s200_joint_2', 'm1n6s200_joint_3', 'm1n6s200_joint_4', 'm1n6s200_joint_5', 'm1n6s200_joint_6']
    finger_joints_names = ['m1n6s200_joint_finger_1', 'm1n6s200_joint_finger_2']
    
    
    client_move_joints = actionlib.SimpleActionClient('/m1n6s200_driver/joints_action/joint_angles',
                                          kinova_msgs.msg.ArmJointAnglesAction)
    client_move_joints.wait_for_server()

    traj = Trajectory()
    rospy.on_shutdown(traj.stop)
    # Command Current Joint Positions first
    first_time = None
    last_time = None
    slower_factor = 1
    print("Reading Rosbag")
    previous_position = 6*[0]
    first_position = 6*[0]
    velocities = 6*[0]
    for topic, msg, t in bag.read_messages(topics=['/joint_states', 'joint_states']):
        positions = msg.position[4:10] #+ (0,0)
        if first_time == None:
            first_time = msg.header.stamp
            first_position = msg.position[4:10]
            previous_position = first_position
            time = slower_factor*(msg.header.stamp - first_time)
        else:   
            time = slower_factor*(msg.header.stamp - first_time)               
        velocities = msg.velocity[4:10]
        #print(positions)
        #print(velocities)
        #print(time.to_sec())
        traj.add_point(positions, velocities, time)  
        last_time = msg.header.stamp      
            
    bag.close()    
    
    #nb = raw_input('Moving to the starting position of the trajectory, press return')   
    
    first_position_degrees = [val_rad*180.0/np.pi for val_rad in first_position] + [0]
    
    #print(first_position_degrees)       
    
    result = joint_position_client(first_position_degrees, 'm1n6s200_')

    #rospy.sleep(20.)
    
    #print("Expected trajectory time: ", slower_factor*(last_time - first_time).to_sec())
    
    #nb = raw_input('Starting trajectory, press return')   
    
    traj.start()
    traj.wait(slower_factor*(last_time - first_time) + rospy.Duration(10))
    print(traj.result())
    print("Exiting - Joint Trajectory Action Complete")


def main():
    """Joint Trajectory Example: Simple Action Client
    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.
    Make sure to start the joint_trajectory_action_server
    first. Then run this example command a short series of trajectory points for the arm
    to follow.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('filename')
    args = parser.parse_args()

    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client")
    print("Running. Ctrl-c to quit")

    replay_rosbag_trajectory(args.filename)

if __name__ == "__main__":
    main()
        
