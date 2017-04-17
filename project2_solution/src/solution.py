#!/usr/bin/env python  
import rospy

import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    object_transform = geometry_msgs.msg.TransformStamped()
    object_transform.header.stamp = rospy.Time.now()
    object_transform.header.frame_id = "base_frame"
    object_transform.child_frame_id = "object_frame"
    object_transform.transform.rotation = tf.transformations.quaternion_from_euler(0.79, 0.0, 0.79)
    object_transform.transform.translation = tf.transformations.translation_matrix((0.0, 1.0, 1.0))
    br.sendTransform(object_transform)

    robot_transform = geometry_msgs.msg.TransformStamped()
    robot_transform.header.stamp = rospy.Time.now()
    robot_transform.header.frame_id = "base_frame"
    robot_transform.child_frame_id = "robot_frame"
    robot_transform.transform.rotation = tf.transformations.quaternion_about_axis(1.5, (0.0, 0.0, 1.0))
    robot_transform.transform.translation = tf.transformations.translation_matrix((0.0, -1.0, 0.0))
    br.sendTransform(robot_transform)
 
    camera_transform = geometry_msgs.msg.TransformStamped()
    camera_transform.header.stamp = rospy.Time.now()
    camera_transform.header.frame_id = "robot_frame"
    camera_transform.child_frame_id = "camera_frame"
    camera_transform.transform.translation = tf.transformations.translation_matrix((0.0, 0.1, 0.1))
    camera_transform.transform.rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    br.sendTransform(camera_transform)

if __name__ == '__main__':

    rospy.init_node('project2_solution')

    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.05)
