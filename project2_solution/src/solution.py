#!/usr/bin/env python  
import rospy

import numpy as np

import tf
import tf2_ros
import geometry_msgs.msg

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
 
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    
    T3 = tf.transformations.translation_matrix((0.0,0.1,0.1))

    '''
    r2 = tf.transformations.inverse_matrix(
        tf.transformations.quaternion_matrix(
        tf.transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))))

    r3 = tf.transformations.concatenate_matrices(
            r2, tf.transformations.quaternion_matrix(
                tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)))

    T3 = tf.transformations.concatenate_matrices(
        tf.transformations.translation_matrix((0.0,0.1,0.1)),r3)
    '''

    '''
    Calculate the vector pointing from the camera to the object, 
    use the dot and cross products to deduce the angle and axis
    to rotate around.
    '''
    # Translate the camera_transform

    camera_transform.transform = message_from_transform(T3)

    #t_object = tf.transformations.translation_from_matrix(object_transform)
    #t_camera = tf.transformations.translation_from_matrix(camera_transform)

    # Calculate the direction of the 3 axis of camera_frame
    x_cam = camera_transform.transform.translation.x
    y_cam = camera_transform.transform.translation.y
    z_cam = camera_transform.transform.translation.z

    # Calculate the displacement in 3D
    '''
    d3 = tf.transformations.translation_matrix((0.0,0.1,0.1))
    d2 = tf.transformations.translation_matrix((0.0,-1.0,0.0))
    d1 = tf.transformations.translation_matrix((0.0,1.0,1.0))

    d = tf.transformations.concatenate_matrices(d3, d2,d1)
    translation = tf.transformations.translation_from_matrix(d)

    v = geometry_msgs.msg.Vector3(
        translation[0],
        translation[1],
        translation[2])
    '''

    x = object_transform.transform.translation.x - x_cam
    y = object_transform.transform.translation.y - y_cam
    z = object_transform.transform.translation.z - z_cam

    # calculate the angle between 'v' and the others

    angle = angle_between([x_cam,y_cam,z_cam],[x,y,z])

    # Moves the tf the angles indicated

    br.sendTransform(camera_transform)


if __name__ == '__main__':

    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
