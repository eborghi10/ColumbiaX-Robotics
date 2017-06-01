#!/usr/bin/env python

import math
import numpy

import geometry_msgs.msg
import moveit_commander
import rospy
import tf

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class ObstacleGenerator(object):

    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()

    def no_obs(self):
        print 'Removing all obstacles'
        self.scene.remove_world_object("obs1")
        self.scene.remove_world_object("obs2")
        self.scene.remove_world_object("obs3")
        self.scene.remove_world_object("obs4")

    def simple_obs(self):
        print 'Adding simple obstacle'
        self.no_obs()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "world_link"
        pose_stamped.header.stamp = rospy.Time(0)

        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.25, 0.2)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,1))

    def complex_obs(self):
        print 'Adding hard obstacle'
        self.no_obs()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "world_link"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.25, 0.4)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.0, 0.8)) )
        self.scene.add_box("obs2", pose_stamped,(0.1,0.5,0.1))

    def super_obs(self):
        print 'Adding super hard obstacle'
        self.no_obs()
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "world_link"
        pose_stamped.header.stamp = rospy.Time(0)
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.25, 0.4)) )
        self.scene.add_box("obs1", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.00, 0.8)) )
        self.scene.add_box("obs2", pose_stamped,(0.1,0.5,0.1))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, -0.25, 0.4)) )
        self.scene.add_box("obs3", pose_stamped,(0.1,0.1,0.8))
        pose_stamped.pose = convert_to_message( tf.transformations.translation_matrix((0.5, 0.0, 0.3)) )
        self.scene.add_box("obs4", pose_stamped,(0.1,0.5,0.1))

