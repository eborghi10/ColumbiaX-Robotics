#!/usr/bin/env python
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

first_time = True

def message_from_transform(T):
	'''
	http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Transform.html

	geometry_msgs/Vector3 translation
	geometry_msgs/Quaternion rotation
	'''
	msg = geometry_msgs.msg.Transform()
	q = tf.transformations.quaternion_from_matrix(T)
	translation = tf.transformations.translation_from_matrix(T)
	msg.translation.x = translation[0]
	msg.translation.y = translation[1]
	msg.translation.z = translation[2]
	msg.rotation.x = q[0]
	msg.rotation.y = q[1]
	msg.rotation.z = q[2]
	msg.rotation.w = q[3]
	return msg

########################################################
# CALCULATES THE ANGLE BETWEEN N-DIMENSIONAL VECTORS
# http://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

def matrix_by_vector_multiplication(matrix,vector):
    """Multiplication of matrix by vector"""
    vector.append(1)
    return [sum([vector[x]*matrix[n][x] for x in range(len(vector))]) for n in range(len(matrix))]

########################################################

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"

    T1 = tf.transformations.concatenate_matrices(
    	tf.transformations.quaternion_matrix(
    		tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)),
    	tf.transformations.translation_matrix((0.0, 1.0, 1.0)))
    object_transform.transform = message_from_transform(T1)

    br.sendTransform(object_transform)

    ########################################################

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"

    T2 = tf.transformations.concatenate_matrices(
    	tf.transformations.quaternion_matrix(
    		tf.transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))),
    	tf.transformations.translation_matrix((0.0, -1.0, 0.0)))

    robot_transform.transform = message_from_transform(T2)

    br.sendTransform(robot_transform)

    ########################################################

    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"

    '''
    Calculate the vector pointing from the camera to the object,
    use the dot and cross products to deduce the angle and axis
    to rotate around.
    '''

    global angle
    global x_axis
    global p2_camera
    global p2_robot
    global first_time

    # Calculates the homogeneous matrix for camera_frame

    # displacement matrix of camera_frame
    D3 = tf.transformations.translation_matrix([0.0, 0.1, 0.1])
    rospy.logdebug("\n\nD3 = %s\n", D3)

    # Only for the first run, we don't rotate the camera_frame
    # because it's an unknown parameter
    if first_time == True:
        first_time = False
        R3 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0,0,0))

	T3 = tf.transformations.concatenate_matrices(D3, R3)
	rospy.logdebug("T3 = %s\n", T3)

	# Calculates the origin of coordinates from object_frame
	# with respect to camera_frame

	# the origin of coordinates of object_frame (p2) is the
	# traslation part of the homogeneous matrix from base_frame
	# to object_frame
	p2 = tf.transformations.translation_from_matrix(T1)

	# p2 referenced to robot_frame
	p2_robot = matrix_by_vector_multiplication(
		tf.transformations.inverse_matrix(T2), p2.tolist())
	p2_robot = p2_robot[:(len(p2_robot)-1)]

	# p2 referenced to camera_frame
	p2_camera = matrix_by_vector_multiplication(
		tf.transformations.inverse_matrix(T3), p2_robot)
	p2_camera = p2_camera[:(len(p2_camera)-1)]

	# x axis of camera_frame viewed from its own frame
	x_axis = [1,0,0]

	# angle difference between x axis and the origin
	# of object_frame
	angle = angle_between(x_axis, p2_camera)
    rospy.logdebug("angle = %s\n",angle*180/3.14159)

    # calculates the vector from which the x axis has to rotate
    v_normal = np.cross(x_axis,p2_camera)

    # with the calculated values, apply the homogeneous matrix
    # to camera_frame

    R3 = tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(
                angle,v_normal))

    T3 = tf.transformations.concatenate_matrices(D3, R3)

    camera_transform.transform = message_from_transform(T3)

    br.sendTransform(camera_transform)

########################################################

if __name__ == '__main__':

    rospy.init_node('project2')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
