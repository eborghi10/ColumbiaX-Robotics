#!/usr/bin/env python

import numpy
import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import moveit_commander
import rospy
import sensor_msgs.msg
import sys
import tf
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from copy import deepcopy

from obstacle_generator import ObstacleGenerator
from obstacle_generator import convert_to_message

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

class MarkerControl(object):

    def __init__(self):
        print "Marker Control initializing..."
        self.og = ObstacleGenerator()

        # Publisher to send commands
        self.pub_command = rospy.Publisher("/motion_planning_goal", geometry_msgs.msg.Transform, 
                                           queue_size=1)        
        # Create interactive marker
        self.init_marker()
        print "Initialization done."

    def control_marker_feedback(self, feedback):
        pass

    def move_arm_cb(self, feedback):
        T = convert_from_message(feedback.pose)
        msg = convert_to_trans_message(T)
        self.pub_command.publish(msg)
        print 'Goal pubished'

    def no_obs_cb(self, feedback):
        self.og.no_obs()

    def simple_obs_cb(self, feedback):
        self.og.simple_obs()

    def complex_obs_cb(self, feedback):
        self.og.complex_obs()

    def super_obs_cb(self, feedback):
        self.og.super_obs()

    def init_marker(self):

        self.server = InteractiveMarkerServer("control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = "/world_link"
        control_marker.name = "move_arm_marker"

        move_control = InteractiveMarkerControl()
        move_control.name = "move_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_y"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "move_z"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control_marker.controls.append(move_control)

        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_x"
        move_control.orientation.w = 1
        move_control.orientation.x = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_y"
        move_control.orientation.w = 1
        move_control.orientation.z = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)
        move_control = InteractiveMarkerControl()
        move_control.name = "rotate_z"
        move_control.orientation.w = 1
        move_control.orientation.y = 1
        move_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control_marker.controls.append(move_control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        menu_control.always_visible = True
        box = Marker()        
        box.type = Marker.CUBE
        box.scale.x = 0.15
        box.scale.y = 0.03
        box.scale.z = 0.03
        box.color.r = 0.5
        box.color.g = 0.5
        box.color.b = 0.5
        box.color.a = 1.0
        menu_control.markers.append(box)
        box2 = deepcopy(box)
        box2.scale.x = 0.03
        box2.scale.z = 0.1
        box2.pose.position.z=0.05
        menu_control.markers.append(box2)
        control_marker.controls.append(menu_control)

        control_marker.scale = 0.25        
        self.server.insert(control_marker, self.control_marker_feedback)

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Move Arm", callback=self.move_arm_cb)
        obs_entry = self.menu_handler.insert("Obstacles")
        self.menu_handler.insert("No Obstacle", callback=self.no_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Simple Obstacle", callback=self.simple_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Hard Obstacle", callback=self.complex_obs_cb, parent=obs_entry)
        self.menu_handler.insert("Super-hard Obstacle", callback=self.super_obs_cb, parent=obs_entry)
        self.menu_handler.apply(self.server, "move_arm_marker",)

        self.server.applyChanges()

        Ttrans = tf.transformations.translation_matrix((0.6,0.2,0.5))
        Rtrans = tf.transformations.rotation_matrix(1.57,(0,1,0))
        self.server.setPose("move_arm_marker", convert_to_message(numpy.dot(Ttrans,Rtrans)))
        self.server.applyChanges()

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('marker_control', anonymous=True)
    mc = MarkerControl()
    rospy.spin()

