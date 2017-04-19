#!/usr/bin/env python  
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

first_time = True

def message_from_transform(T):
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
    
    v3 = [0.0, 0.1, 0.1]
    D3 = tf.transformations.translation_matrix(v3)
    rospy.loginfo("\n\nD3 = %s\n", D3)

    global first_time
    if first_time == True:
        first_time = False
        R3 = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0,0,0))            
    else:
        global R5
        R3 = R5

    T3 = tf.transformations.concatenate_matrices(D3, R3)
    rospy.loginfo("T3 = %s\n", T3)

    # Nodo 4 referenciado al nodo 1
    
    # Robotica John Craig 3ra edicion, ejemplo 2.2 - p.29
    v_4 = matrix_by_vector_multiplication(T2,v3)

    v_4 = v_4[:(len(v_4)-1)]

    rospy.loginfo("\n\nT2 = %s\n\nv_4 = %s\n", T2,v_4)

    # Vector p4->p2

    v_2 = tf.transformations.translation_from_matrix(T1)

    v_2_4 = v_2 - v_4

    rospy.loginfo("V2 = %s\n\nV2_V4 = %s\n",v_2,v_2_4)

    # Angulo de rotacion entre el eje x de camera_frame (eje x 
    # de robot_frame) y el origen de object_frame
    
    # [1,0,0]: Versor x del camera_frame
    x_robot = matrix_by_vector_multiplication(T3,[1.0, 0.0, 0.0])
    x_robot = matrix_by_vector_multiplication(T2,x_robot[:(len(x_robot)-1)])
    x_robot = x_robot[:(len(x_robot)-1)]
    
    rospy.loginfo("x_robot = %s\n", x_robot)

    angle = angle_between(x_robot, v_2_4)  
    rospy.loginfo("angle = %s\n",angle*180/3.14159)  

    #angle2 = tf.transformations.angle_between_vectors(x_robot, v_2_4)  
    #rospy.loginfo("angle2 = %s",angle2*180/3.14159)  

    # Versor normal a x y versor (producto vectorial)
    # Es el eje de rotacion del sistema camera_frame

    v_normal = np.cross(x_robot,v_2_4)

    rospy.loginfo("v_normal = %s\n", v_normal)

    # Aplico la rotacion

    R5 = tf.transformations.quaternion_matrix(
            tf.transformations.quaternion_about_axis(
                angle,v_normal))

    T3 = tf.transformations.concatenate_matrices(D3, R5)
    #rospy.loginfo("\n\nT3 = %s\n\nD3 = %s\n\nR5 = %s\n", T3,D3,R5)

    camera_transform.transform = message_from_transform(T3)

    '''
    Calculate the vector pointing from the camera to the object, 
    use the dot and cross products to deduce the angle and axis
    to rotate around.
    '''

    br.sendTransform(camera_transform)

########################################################

if __name__ == '__main__':

    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
