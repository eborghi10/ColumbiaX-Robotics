#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)
    #-------------------- Fill in your code here ---------------------------
    rospy.loginfo('\n\nnum_joints\t%s\n\n', num_joints)
    # Prints the arguments for debugging
    # joint_transforms: list containing the transforms of all the joints with
    # respect to the base frame
    #rospy.loginfo('\n\njoint_transforms\n\n %s\n\n', joint_transforms)
    #rospy.loginfo('\n\nb_T_ee_current\n\n %s\n\n', b_T_ee_current)
    #rospy.loginfo('\n\nb_T_ee_desired\n\n %s\n\n', b_T_ee_desired)

    # compute the desired change in end-effector pose from b_T_ee_current to b_T_ee_desired
    
    # DX = inv(b_T_ee_current) * b_T_ee_Desired

    b_t_ee = tf.transformations.translation_from_matrix(b_T_ee_current) - tf.transformations.translation_from_matrix(b_T_ee_desired)
    rospy.loginfo('\n\nb_t_ee\t%s\n\n', b_t_ee)

    angle, axis = rotation_from_matrix(b_T_ee_current)
    rospy.loginfo('\n\nangle\t%s\taxis\t%s\n\n', angle,axis)
    b_R_ee = numpy.dot(angle,axis)
    rospy.loginfo('\n\nb_R_ee\t%s\n\n', b_R_ee)

    angle2, axis2 = rotation_from_matrix(b_T_ee_desired)
    rospy.loginfo('\n\nangle2\t%s\taxis2\t%s\n\n', angle2,axis2)
    b_R_ee_2 = numpy.dot(angle2,axis2)
    rospy.loginfo('\n\nb_R_ee_2\t%s\n\n', b_R_ee_2)

    # TODO: Can be done in one step?
    b_R_ee = b_R_ee - b_R_ee_2

    '''
    https://courses.edx.org/courses/course-v1:ColumbiaX+CSMM.103x+1T2017/discussion/forum/61ec2db861132ac377a4e036455725759857558e/threads/5928eb391305f108260008bc
    '''

    # V_ee
    # To obtain ee_R_b = (b_R_ee)^-1 = (b_R_ee).T because rotation matrices are orthogonal.
    # Extract rotation of b_T_ee_current and Transpose.
    # I used 'sxyz' instead for euler_from_matrix

    # convert the desired change into a desired end-effector velocity
    # (the simplest form is to use a PROPORTIONAL CONTROLLER)
    proportional_gain = 1000
    x_dot = proportional_gain * b_t_ee

    # normalize the desired change
    # OTHER ALTERNATIVE: https://stackoverflow.com/a/27903986
    x_dot_norm = x_dot / sum(x_dot)
    rospy.loginfo('\n\nx_dot_norm\t%s\n\n', x_dot_norm)

    # numerically compute the robot Jacobian. For each joint compute the matrix
    # that relates the velocity of that joint to the velocity of the end-effector
    # in its own coordinate frame. Assemble the last column of all these matrices
    # to construct the Jacobian.

    # TODO: IMPLEMENT
    # rotation: j_T_ee = (b_T_j)^-1*b_T_ee

    # J = [V_0[:,5]*dq[0] ... V_n-1[:,5]*dq[n-1]]

    # This tells you what a specific joint is going to do to the end effector,
    # in the reference frame of the joint
    # take the inverse of b_T_j = j_T_b and then do j_T_b.b_T_ee = j_T_ee.

    # ee_T_j = dot(ee_T_b, ith joint(b_T_j :: joint_transforms :: from_base_to_joint))
    # The rotation part is multiplied by S_matrix(translation part)
    # -> get the last column and put into V_j
    for i in num_joints:
    	numpy.dot(-tf.transformations.rotation_from_matrix(joint_transforms[num_joints]),
    		S_matrix(tf.transformations.translation_from_matrix(joint_transforms[num_joints])))

    J = numpy.vstack((
    	numpy.cross(x_dot_norm - t1, t1),
    	numpy.cross(x_dot_norm - t2, t2),
    	numpy.cross(x_dot_norm - t3, t3),
    	numpy.cross(x_dot_norm - t4, t4),
    	numpy.cross(x_dot_norm - t5, t5),
    	numpy.cross(x_dot_norm - t6, t6))).T
    rospy.loginfo('\n\nJacobian\n\n%s\n\n', J)
	
    # Compute the pseudo-inverse of the Jacobian. Make sure to avoid numerical
    # issues that can arise from small singular values
    J_pinv = numpy.linalg.pinv(J, rcond=1e-15)
    rospy.loginfo('\n\nJacobian Pseudo-inverse\n\n%s\n\n', J_pinv)

    # Use the pseudo-inverse of the Jacobian to map from end-effector velocity to
    # joint velocities. You might want to scale these joint velocities such that
    # their norm (or their largest element) is lower than a certain threshold

    # dq = J_pinv * V_ee
    dq = numpy.dot(J_pinv, x_dot_norm)
    rospy.loginfo('\n\ndq\n\n%s\n\n', dq)

    if red_control == True:
    	# implements the null-space control on the first joint

        # find a joint velocity that brings the joint closer to the secondary objective.

        # use the Jacobian and its pseudo-inverse to project this velocity into the
        # Jacobian nullspace. Be careful to use the 'exact' version of the Jacobian
        # pseudo-inverse, not its 'safe' version. 

        #Then add the result to the joint velocities obtained for the primary objective
    	pass

    #----------------------------------------------------------------------
    return dq
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

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

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(7)
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
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
    cc = CartesianControl()
    rospy.spin()
