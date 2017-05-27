#!/usr/bin/env python

import math
import numpy
from threading import Thread, Lock

import geometry_msgs.msg
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
import rospy
from sensor_msgs.msg import JointState
import tf
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

from grader import CartesianGrader
    
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

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

class MarkerControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        # Publishes Cartesian goals
        self.pub_command = rospy.Publisher("/cartesian_command", geometry_msgs.msg.Transform, 
                                           queue_size=1)

        # Publishes Redundancy goals
        self.pub_red = rospy.Publisher("/redundancy_command", Float32, queue_size=1)

        # Publisher to set robot position
        self.pub_reset = rospy.Publisher("/joint_command", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()

        #Create "Interactive Marker" that we can manipulate in RViz
        self.init_marker()
        self.ee_tracking = 0
        self.red_tracking = 0
        self.q_current = []

        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0

        self.mutex = Lock()        
        self.timer = rospy.Timer(rospy.Duration(0.3), self.timer_callback)

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.joint_callback)

    #Resets the robot to a known pose
    def reset_robot(self):
        cmd = JointState()
        cmd.position.append(0.0)
        cmd.position.append(1.0)
        cmd.position.append(0.0)
        cmd.position.append(1.0)
        cmd.position.append(0.0)
        cmd.position.append(0.0)
        cmd.position.append(0.0)
        self.pub_reset.publish(cmd)
        rospy.sleep(1.0)

    def init_marker(self):

        self.server = InteractiveMarkerServer("control_markers")

        control_marker = InteractiveMarker()
        control_marker.header.frame_id = "/world_link"
        control_marker.name = "cc_marker"

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
        control_marker.controls.append(menu_control)

        control_marker.scale = 0.25
        self.server.insert(control_marker, self.control_marker_feedback)

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Run grader", callback=self.run_grader_cb)

        self.menu_handler.apply(self.server, "cc_marker",)

        redundancy_marker = InteractiveMarker()
        redundancy_marker.header.frame_id = "/lwr_arm_1_link"
        redundancy_marker.name = "red_marker"
        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_z"
        rotate_control.orientation.w = 1
        rotate_control.orientation.y = 1
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        redundancy_marker.controls.append(rotate_control)
        redundancy_marker.scale = 0.25
        self.server.insert(redundancy_marker, self.redundancy_marker_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def update_marker(self, T):
        # Note that we keep marker orientation fixed so that robot can be taken out of
        # singularity even when only doing translation only control
        # Otherwise, arrows always point along arm, and arm stays stretched out
        # forever if it starts out that way.
        # Ttrans = tf.transformations.translation_matrix(
        #     tf.transformations.translation_from_matrix(T)
        # )
        self.server.setPose("cc_marker", convert_to_message(T))
        self.server.applyChanges()

    def redundancy_marker_feedback(self, feedback):       
        if feedback.event_type == feedback.MOUSE_DOWN:
            self.x_target = self.x_current
            self.red_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            self.q0_desired = self.q_current[0]
            self.red_tracking = 0
        if feedback.event_type == feedback.POSE_UPDATE:
            q = feedback.pose.orientation
            qvec = ((q.x, q.y, q.z, q.w))
            R = tf.transformations.quaternion_matrix(qvec)
            angle, direction, point = tf.transformations.rotation_from_matrix(R)
            self.mutex.acquire()
            if abs(self.q0_desired - angle) < 1.0: self.q0_desired = angle
            self.mutex.release()

    def control_marker_feedback(self, feedback):
        if feedback.event_type == feedback.MOUSE_DOWN:
            # since default marker orientation stays fixed, change in orientation
            # must be applied relative to reference orientation when we started dragging
            self.R_base = self.x_current
            self.R_base[0:3,3]=0
            self.ee_tracking = 1
        elif feedback.event_type == feedback.MOUSE_UP:
            self.ee_tracking = 0
            self.x_target = self.x_current
        elif feedback.event_type == feedback.POSE_UPDATE:
            self.mutex.acquire()
            R = tf.transformations.quaternion_matrix((feedback.pose.orientation.x,
                                                      feedback.pose.orientation.y,
                                                      feedback.pose.orientation.z,
                                                      feedback.pose.orientation.w))
            T = tf.transformations.translation_matrix((feedback.pose.position.x, 
                                                       feedback.pose.position.y, 
                                                       feedback.pose.position.z))
            #self.x_target = numpy.dot( T,numpy.dot(R,self.R_base) )
            self.x_target = numpy.dot( T,R )
            self.mutex.release()

    def run_grader_cb(self, feedback):
        print "Initializing"
        cg = CartesianGrader(self.server)
        rospy.sleep(1.0)
        print "Resetting robot"
        self.reset_robot()
        rospy.sleep(1.0)

        print "Moving with translation only"
        trans = tf.transformations.translation_matrix((0.3, 0.2, 1.0))
        rot = tf.transformations.quaternion_matrix((0.0, 0.0, 0.0, 1))
        cg.go_to_pose("Translation", numpy.dot(trans,rot),10)

        print "Moving with rotation only"
        trans = tf.transformations.translation_matrix((0.3, 0.2, 1.0))
        rot = tf.transformations.quaternion_matrix((0.0, 0.0, 1.0, 0.0))
        cg.go_to_pose("Rotation", numpy.dot(trans,rot),10)

        print "Moving with both translation and rotation"
        trans = tf.transformations.translation_matrix((0.5, 0.1, 0.8))
        rot = tf.transformations.quaternion_matrix((0.0, 0.47, 0.0, 0.87))
        cg.go_to_pose("Translation and Rotation", numpy.dot(trans,rot),10)

        self.server.erase("cg_marker")
        self.server.applyChanges()

    def timer_callback(self, event):
        if self.ee_tracking:
            msg = JointState()
            self.mutex.acquire()
            msg = convert_to_trans_message(self.x_target)
            self.pub_command.publish(msg)
            self.mutex.release()
        if self.red_tracking:
            red = Float32()
            self.mutex.acquire()
            red.data = self.q0_desired
            self.pub_red.publish(red)
            self.mutex.release()

    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        if not self.ee_tracking:
            self.update_marker(self.x_current)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def process_link_recursive(self, link, T, joint_values):
        if link not in self.robot.child_map: 
            self.x_current = T
            return
        for i in range(0,len(self.robot.child_map[link])):
            (joint_name, next_link) = self.robot.child_map[link][i]
            if joint_name not in self.robot.joint_map:
                rospy.logerror("Joint not found in map")
                continue
            current_joint = self.robot.joint_map[joint_name]        

            trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                  current_joint.origin.xyz[1],
                                                                  current_joint.origin.xyz[2]))
            rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                         current_joint.origin.rpy[1],
                                                         current_joint.origin.rpy[2], 'rxyz')
            origin_T = numpy.dot(trans_matrix, rot_matrix)
            current_joint_T = numpy.dot(T, origin_T)
            if current_joint.type != 'fixed':
                if current_joint.name not in joint_values.name:
                    rospy.logerror("Joint not found in list")
                    continue
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)       

if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    print "Initializing Marker control"
    mc = MarkerControl()
    rospy.sleep(1.0)
    print "Resetting robot"
    mc.reset_robot()
    rospy.sleep(1.0)
    print "Ready"
    rospy.spin()
