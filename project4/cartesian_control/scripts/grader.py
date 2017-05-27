#!/usr/bin/env python
import numpy
import rospy
import tf
import time

from sensor_msgs.msg import JointState
import geometry_msgs.msg
from geometry_msgs.msg import Transform
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import *

def convert_to_trans_message(T):
    t = Transform()
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

def is_same(matrix0, matrix1):
    matrix0 = numpy.array(matrix0, dtype=numpy.float64, copy=True)
    matrix0 /= matrix0[3, 3]
    matrix1 = numpy.array(matrix1, dtype=numpy.float64, copy=True)
    matrix1 /= matrix1[3, 3]
    return numpy.allclose(matrix0, matrix1, 0, 1e-2)

class CartesianGrader(object):

    #Initialization
    def __init__(self, server):
        
        # Publisher to set robot position
        self.pub_reset = rospy.Publisher("/joint_command", JointState, queue_size=1)

        #Publisher to issue commands
        self.pub_cmd = rospy.Publisher("/cartesian_command", Transform, queue_size=1) 

        self.server = server

        self.init_marker()
        self.listener = tf.TransformListener()

    def go_to_pose(self, name, T, timeout):
        msg = convert_to_trans_message(T)
        self.update_marker(T)
        start_time = time.time()
        done = False
        while not done and not rospy.is_shutdown():
            self.pub_cmd.publish(msg)
            try:
                (trans,rot) = self.listener.lookupTransform('/world_link','lwr_arm_7_link',
                                                            rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "TF Exception!"
                continue

            TR = numpy.dot(tf.transformations.translation_matrix(trans), 
                           tf.transformations.quaternion_matrix(rot))
            
            if (is_same(T, TR)): 
                print name + ": Reached desired pose"
                done = True

            if (time.time() - start_time > timeout) :
                print name + ": Robot took too long to reach desired pose"
                done = True
            else:
                rospy.sleep(0.1)

    def init_marker(self):

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = "/world_link"
        control_marker.name = "cg_marker"

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

        control_marker.scale = 0.25
        self.server.insert(control_marker, self.control_marker_feedback)

        self.server.applyChanges()

    def control_marker_feedback(self, feedback):
        print "When running the grader, the interactive marker is only meant to display the taret pose. Interacting with it has no effect"

    def update_marker(self, T):
        self.server.setPose("cg_marker", convert_to_message(T))
        self.server.applyChanges()