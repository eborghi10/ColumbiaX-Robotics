#!/usr/bin/env python

import numpy
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import moveit_commander
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import sys
import tf
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from sensor_msgs.msg import JointState
from copy import deepcopy

from obstacle_generator import ObstacleGenerator
from obstacle_generator import convert_to_message
import time

def convert_to_trans_message(T):
    t = geometry_msgs.msg.Transform()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.translation.x = position[0]
    t.translation.y = position[1]
    t.translation.z = position[2]
    t.rotation.x = orientation[0]
    t.rotation.y = orientation[1]
    t.rotation.z = orientation[2]
    t.rotation.w = orientation[3]        
    return t

def convert_from_message(msg):
    R = tf.transformations.quaternion_matrix((msg.orientation.x,
                                              msg.orientation.y,
                                              msg.orientation.z,
                                              msg.orientation.w))
    T = tf.transformations.translation_matrix((msg.position.x, 
                                               msg.position.y, 
                                               msg.position.z))
    return numpy.dot(T,R)

def is_same(matrix0, matrix1):
    matrix0 = numpy.array(matrix0, dtype=numpy.float64, copy=True)
    matrix0 /= matrix0[3, 3]
    matrix1 = numpy.array(matrix1, dtype=numpy.float64, copy=True)
    matrix1 /= matrix1[3, 3]
    return numpy.allclose(matrix0, matrix1, 0, 2e-2)

class Grader(object):

    def __init__(self, server):
        self.server = server

        self.mutex = Lock()

        # Publisher to send commands
        self.pub_command = rospy.Publisher("/motion_planning_goal", geometry_msgs.msg.Transform, 
                                           queue_size=1)        
        self.listener = tf.TransformListener()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, 
                         self.joint_states_callback)

        # Publisher to set robot position
        self.pub_reset = rospy.Publisher("/joint_command", JointState, queue_size=1)
        rospy.sleep(0.5)

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        
        self.reset_robot()

    def joint_states_callback(self, joint_state):
        self.mutex.acquire()
        self.joint_state = joint_state
        self.mutex.release()

    def check_validity(self, joint_state):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = "lwr_arm"
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state = joint_state
        res = self.state_valid_service(req)
        return res.valid
       
        #Resets the robot to a known pose
    def reset_robot(self):
        cmd = JointState()
        cmd.position.append(0.35)
        cmd.position.append(2.04)
        cmd.position.append(-1.35)
        cmd.position.append(1.03)
        cmd.position.append(-0.53)
        cmd.position.append(1.34)
        cmd.position.append(1.64)
        self.pub_reset.publish(cmd)
        rospy.sleep(1.0)

    def goto_pose(self, name, T, timeout):
        self.server.setPose("move_arm_marker", convert_to_message(T))
        self.server.applyChanges()
        self.pub_command.publish(convert_to_trans_message(T))
        print 'Goal published'        
        start_time = time.time()
        done = False
        while not done and not rospy.is_shutdown():

            self.mutex.acquire()
            last_joint_state = deepcopy(self.joint_state)
            self.mutex.release()
            if not self.check_validity(last_joint_state):
                print 'COLLISION!'

            try:
                (trans,rot) = self.listener.lookupTransform('world_link','lwr_arm_7_link',
                                                            rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "TF Exception!"
                continue

            TR = numpy.dot(tf.transformations.translation_matrix(trans), 
                           tf.transformations.quaternion_matrix(rot))
            
            if (is_same(T, TR)): 
                print name + ": Reached goal"
                done = True

            if (time.time() - start_time > timeout) :
                done = True
                print name + ": Robot took too long to reach goal. Grader timed out"
            else:
                rospy.sleep(0.05)