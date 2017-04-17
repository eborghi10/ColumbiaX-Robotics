#!/usr/bin/env python

'''
To run code:

$ roscore
$ rosrun rviz rviz
$ rosrun tf2_examples tf2_examples.py

Axis in rviz:
[X] : red
[Y] : green
[Z] : blue
'''

import rospy

# For matrix multiplications
#import numpy

import tf
import tf2_ros
import geometry_msgs.msg

'''
geometry_msgs.msg.Transform():

- Vector3 translation : float64 {x,y,z}
- Quaternion rotation : float64 {x,y,z,w}
'''

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

def publish_transforms():
	'''
	Transformation between [world] and [F1]:

	- translation: X and Y
	- rotation: euler angles (1,1,1)
	'''
	T1 = tf.transformations.concatenate_matrices(
		tf.transformations.translation_matrix((1.0, 1.0, 0.0)),
		tf.transformations.quaternion_matrix(
			tf.transformations.quaternion_from_euler(1.0, 1.0, 1.0))
		)
	T1_stamped = geometry_msgs.msg.TransformStamped()
	T1_stamped.header.stamp = rospy.Time.now()
	T1_stamped.header.frame_id = "world"
	T1_stamped.child_frame_id = "F1"
	T1_stamped.transform = message_from_transform(T1)
	br.sendTransform(T1_stamped)

	'''
	Transformation between [F1] and [F2]:

	- translation: X
	- rotation: 90 degrees (pi/2 = 1.57 rad)
	'''
	T2 = tf.transformations.concatenate_matrices(
		tf.transformations.translation_matrix((1.0, 0.0, 0.0)),
		tf.transformations.quaternion_matrix(
			tf.transformations.quaternion_about_axis(1.57, (1.0, 0.0, 0.0)))
		)
	T2_stamped = geometry_msgs.msg.TransformStamped()
	T2_stamped.header.stamp = rospy.Time.now()
	T2_stamped.header.frame_id = "F1"
	T2_stamped.child_frame_id = "F2"
	T2_stamped.transform = message_from_transform(T2)
	br.sendTransform(T2_stamped)

	'''
	Translation between [F2] and [F3]:

	It applies the inverse of T2, so the result will end up 
	in F1.
	'''
	T2_inverse = tf.transformations.inverse_matrix(T2)
	T3_stamped = geometry_msgs.msg.TransformStamped()
	T3_stamped.header.stamp = rospy.Time.now()
	T3_stamped.header.frame_id = "F2"
	T3_stamped.child_frame_id = "F3"
	T3_stamped.transform = message_from_transform(T2_inverse)
	br.sendTransform(T3_stamped)

	'''
	Translation between [F3] and [F4]:

	The same as T3 but inverting T1, so it ends up in the world frame.
	'''
	T1_inverse = tf.transformations.inverse_matrix(T1)
	T4_stamped = geometry_msgs.msg.TransformStamped()
	T4_stamped.header.stamp = rospy.Time.now()
	T4_stamped.header.frame_id = "F3"
	T4_stamped.child_frame_id = "F4"
	T4_stamped.transform = message_from_transform(T1_inverse)
	br.sendTransform(T4_stamped)

	'''
	In summary: T1*T2*inv(T2)*inv(T1), so, F4 = world
	'''

if __name__ == '__main__':
	
	rospy.init_node('tf2_examples')

	# Broadcast transformations within a node...
	br = tf2_ros.TransformBroadcaster()
	# ...for each half a second
	rospy.sleep(0.5)

	while not rospy.is_shutdown():
		publish_transforms()
		rospy.sleep(0.5)