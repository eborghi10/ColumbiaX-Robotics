#!/usr/bin/env python

from copy import deepcopy
import math
import numpy
import random
from threading import Thread, Lock
import sys

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import sensor_msgs.msg
import tf
import trajectory_msgs.msg

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

def convert_from_message(msg):
	R = tf.transformations.quaternion_matrix((msg.orientation.x,
											  msg.orientation.y,
											  msg.orientation.z,
											  msg.orientation.w))
	T = tf.transformations.translation_matrix((msg.position.x, 
											   msg.position.y, 
											   msg.position.z))
	return numpy.dot(T,R)

def convert_from_trans_message(msg):
	R = tf.transformations.quaternion_matrix((msg.rotation.x,
											  msg.rotation.y,
											  msg.rotation.z,
											  msg.rotation.w))
	T = tf.transformations.translation_matrix((msg.translation.x, 
											   msg.translation.y, 
											   msg.translation.z))
	return numpy.dot(T,R)

class MoveArm(object):

	def __init__(self):
		print "Motion Planning Initializing..."
		# Prepare the mutex for synchronization
		self.mutex = Lock()

		# Some info and conventions about the robot that we hard-code in here
		# min and max joint values are not read in Python urdf, so we must hard-code them here
		self.num_joints = 7
		self.q_min = []
		self.q_max = []
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		self.q_min.append(-3.14159);self.q_max.append(3.14159)
		# How finely to sample each joint
		self.q_sample = [0.05, 0.05, 0.05, 0.1, 0.1, 0.1, 0.1]
		self.joint_names = ["lwr_arm_0_joint",
							"lwr_arm_1_joint",
							"lwr_arm_2_joint",
							"lwr_arm_3_joint",
							"lwr_arm_4_joint",
							"lwr_arm_5_joint",
							"lwr_arm_6_joint"]

		# Subscribes to information about what the current joint values are.
		rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, 
						 self.joint_states_callback)

		# Subscribe to command for motion planning goal
		rospy.Subscriber("/motion_planning_goal", geometry_msgs.msg.Transform,
						 self.move_arm_cb)

		# Publish trajectory command
		self.pub_trajectory = rospy.Publisher("/joint_trajectory", trajectory_msgs.msg.JointTrajectory, 
											  queue_size=1)        

		# Initialize variables
		self.joint_state = sensor_msgs.msg.JointState()

		# Wait for moveit IK service
		rospy.wait_for_service("compute_ik")
		self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
		print "IK service ready"

		# Wait for validity check service
		rospy.wait_for_service("check_state_validity")
		self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
													  moveit_msgs.srv.GetStateValidity)
		print "State validity service ready"

		# Initialize MoveIt
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group_name = "lwr_arm"
		self.group = moveit_commander.MoveGroupCommander(self.group_name) 
		print "MoveIt! interface ready"

		# Options
		self.subsample_trajectory = True
		print "Initialization done."

	def get_joint_val(self, joint_state, name):
		if name not in joint_state.name:
			print "ERROR: joint name not found"
			return 0
		i = joint_state.name.index(name)
		return joint_state.position[i]

	def set_joint_val(self, joint_state, q, name):
		if name not in joint_state.name:
			print "ERROR: joint name not found"
		i = joint_state.name.index(name)
		joint_state.position[i] = q

	""" Given a complete joint_state data structure, this function finds the values for 
	our arm's set of joints in a particular order and returns a list q[] containing just 
	those values.
	"""
	def q_from_joint_state(self, joint_state):
		q = []
		for i in range(0,self.num_joints):
			q.append(self.get_joint_val(joint_state, self.joint_names[i]))
		return q

	""" Given a list q[] of joint values and an already populated joint_state, this 
	function assumes that the passed in values are for a our arm's set of joints in 
	a particular order and edits the joint_state data structure to set the values 
	to the ones passed in.
	"""
	def joint_state_from_q(self, joint_state, q):
		for i in range(0,self.num_joints):
			self.set_joint_val(joint_state, q[i], self.joint_names[i])

	""" This function will perform IK for a given transform T of the end-effector. It 
	returns a list q[] of 7 values, which are the result positions for the 7 joints of 
	the left arm, ordered from proximal to distal. If no IK solution is found, it 
	returns an empy list.
	"""
	def IK(self, T_goal):
		req = moveit_msgs.srv.GetPositionIKRequest()
		req.ik_request.group_name = self.group_name
		req.ik_request.robot_state = moveit_msgs.msg.RobotState()
		req.ik_request.robot_state.joint_state = self.joint_state
		req.ik_request.avoid_collisions = True
		req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
		req.ik_request.pose_stamped.header.frame_id = "world_link"
		req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
		req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
		req.ik_request.timeout = rospy.Duration(3.0)
		res = self.ik_service(req)
		q = []
		if res.error_code.val == res.error_code.SUCCESS:
			q = self.q_from_joint_state(res.solution.joint_state)
		return q

	""" This function checks if a set of joint angles q[] creates a valid state, or 
	one that is free of collisions. The values in q[] are assumed to be values for 
	the joints of the left arm, ordered from proximal to distal. 
	"""
	def is_state_valid(self, q):
		req = moveit_msgs.srv.GetStateValidityRequest()
		req.group_name = self.group_name
		current_joint_state = deepcopy(self.joint_state)
		current_joint_state.position = list(current_joint_state.position)
		self.joint_state_from_q(current_joint_state, q)
		req.robot_state = moveit_msgs.msg.RobotState()
		req.robot_state.joint_state = current_joint_state
		res = self.state_valid_service(req)
		return res.valid

	def check_collision(self, vector, q):
		# v = b-a
		# s = v / n
		# discrete path: a + k * s | 1 < k <  n
		
		vector /= numpy.linalg.norm(vector)
		#vector *= 0.05
		
		n = max(numpy.absolute(map(int, self.lists_div(vector, self.q_sample))))
		#rospy.loginfo('\n\n[n]\t%s\n\n', n)
		step = self.list_div(min(self.q_sample), vector)
		#rospy.loginfo('\n\n[step]\t%s\n\n', step)

		path = numpy.outer(numpy.arange(1, n), step) + q
		
		for i in range(path.shape[0]):
			if self.is_state_valid(path[i]) == False:
				return False
		return True

	def lists_sub(self, v1, v2):
		return [v1[i]-v2[i] for i in range(self.num_joints)]

	def lists_norm(self, v1, v2):
		return numpy.linalg.norm(self.lists_sub(v1,v2))

	def lists_div(self, v1, v2):
		return [v1[i]/v2[i] for i in range(self.num_joints)]

	def list_div(self, v1, v2):
		return [v1/v2[i] for i in range(self.num_joints)]
			
			   
	def motion_plan(self, q_start, q_goal, q_min, q_max):
		# q_start: list of joint values of the robot at the starting position.
		# This is the position in CONFIGURATION SPACE from which to start.
		rospy.loginfo('\n\n[q_start]\t%s\n\n', q_start)
		# q_goal: list of joint values of the robot at the goal position.
		# This is the position in configuration space for which to plan.
		rospy.loginfo('\n\n[q_goal]\t%s\n\n', q_goal)
		# q_min: list of lower joint limits
		#rospy.loginfo('\n\n[q_min]\t%s\n\n', q_min)
		# q_max: list of upper joint limits
		#rospy.loginfo('\n\n[q_max]\t%s\n\n', q_max)

		# Create an RRT node object. This object must hold both a position in
		# configuration space and a reference to its parent node. You can then
		# store each new node in a list
		rrt_object = {"position_in_config_space" : q_start, "parent_node" : -1}
		rrt_list = []
		rrt_list.append(rrt_object)
		rospy.loginfo('\n\n[RRT list]\n\n%s\n\n', rrt_list)
		
		# The main part of the algorithm is a loop, in which you expand the
		# tree until it reaches the goal. You might also want to include some additional
		# exit conditions (maximum number of nodes, a time-out) such that your algorithm
		# does not run forever on a problem that might be impossible to solve.
		maximum_nodes = 150
		maximum_time_secs = 240
		begin = rospy.get_rostime().secs
		now = rospy.get_rostime().secs

		while (len(rrt_list) < maximum_nodes) or ((now - begin) < maximum_time_secs):
			# Sample a random point in configuration space within the joint limits.
			# You can use the random.random() function provided by Python. Remember that
			# a "point" in configuration space must specify a value for each robot joint,
			# and is thus 7-dimensional (in the case of this robot)!
			q_random = [random.uniform(q_min[i], q_max[i]) for i in range(self.num_joints)]
			rospy.loginfo('\n\n[q random]\t%s\n\n', q_random)

			# Find the node already in your tree that is closest to this random point.
			distances = [self.lists_norm(q_pos, q_random) for i,q_pos in enumerate(d["position_in_config_space"] for d in rrt_list)]
			rospy.loginfo('\n\n[distances]\t%s\n\n', distances)

			min_distance_index = distances.index(min(distances))
			rospy.loginfo('\n\n[min distance index]\t%s\n\n', min_distance_index)

			# Find the point that lies a predefined distance (e.g. 0.5) from this existing
			# node in the direction of the random point.
			vector = self.lists_sub(rrt_list[min_distance_index].get("position_in_config_space"), q_random)
			rospy.loginfo('\n\n[min vector]\t%s\n\n', vector)

			# Check if the path from the closest node to this point is collision free.
			# To do so you must discretize the path and check the resulting points along
			# the path. You can use the is_state_valid method to do so. The MoveArm class
			# has a member q_sample - a list that defines the minimum discretization for
			# each joint. You must make sure that you sample finely enough that this minimum
			# is respected for each joint.

			if self.check_collision(vector, q_random) == True:
				# If the path is collision free, add a new node with at the position of the
				# point and with the closest node as a parent.

				rrt_object.update({"position_in_config_space" : q_random})
				# parent_node = min_distance_index
				rrt_object.update({"parent_node": min_distance_index})
				rrt_list.append(rrt_object)

				# Check if the path from this new node to the goal is collision free.
				# If so, add the goal as a node with the new node as a parent. The tree
				# is complete and the loop can be exited.
				vector = [q_random[i] - q_goal[i] for i in range(0, self.num_joints)]
				
				if self.check_collision(vector, q_random) == True:
					parent_node = rrt_list[-1].get("parent_node")
					rrt_object.update({"parent_node" : parent_node})
					rrt_object.update({"position_in_config_space" : q_goal})
					rospy.loginfo('\n\n[] goal node reached\n\n')
					break
				else:
					rospy.loginfo('\n\n[2] invalid state\n\n')
			else:
				rospy.loginfo('\n\n[1] invalid state\n\n')

			rospy.loginfo('\n\n[RRT list]\n\n%s\n\n', rrt_list)

			rospy.loginfo('\n\n[len(RRT list)]\t%s\n\n', len(rrt_list))
			now = rospy.get_rostime().secs
			rospy.loginfo('\n\n[time]\t%s\n\n', now-begin)
		
		# Trace the tree back from the goal to the root and for each
		# node insert the position in configuration space to a list of
		# joints values.
		q_list = [q_goal]
		parent_node = rrt_list[-1].get("parent_node")

		while True:

			q_list.insert(0, rrt_list[parent_node].get("position_in_config_space"))

			if parent_node == -1:
				break
			else:
				parent_node = rrt_list[parent_node].get("parent_node")

		# As we have been following the branches of the tree the path
		# computed this way can be very coarse and more complicated
		# than necessary. Therefore, you must check this list of joint
		# values for shortcuts. Similarly to what you were doing when
		# constructing the tree, you can check if the path between any
		# two points in this list is collision free. You can delete any
		# points between two points connected by a collision free path.
		
		# Return the resulting trimmed path
		#q_list = [q_start, q_goal]
		rospy.loginfo('\n\n[q list size]\t%s\n\n', len(q_list))
		rospy.loginfo('\n\n[q list]\n\n%s\n\n', q_list)
		return q_list

	def create_trajectory(self, q_list, v_list, a_list, t):
		joint_trajectory = trajectory_msgs.msg.JointTrajectory()
		for i in range(0, len(q_list)):
			point = trajectory_msgs.msg.JointTrajectoryPoint()
			point.positions = list(q_list[i])
			point.velocities = list(v_list[i])
			point.accelerations = list(a_list[i])
			point.time_from_start = rospy.Duration(t[i])
			joint_trajectory.points.append(point)
		joint_trajectory.joint_names = self.joint_names
		return joint_trajectory

	def create_trajectory(self, q_list):
		joint_trajectory = trajectory_msgs.msg.JointTrajectory()
		for i in range(0, len(q_list)):
			point = trajectory_msgs.msg.JointTrajectoryPoint()
			point.positions = list(q_list[i])
			joint_trajectory.points.append(point)
		joint_trajectory.joint_names = self.joint_names
		return joint_trajectory

	def project_plan(self, q_start, q_goal, q_min, q_max):
		q_list = self.motion_plan(q_start, q_goal, q_min, q_max)
		joint_trajectory = self.create_trajectory(q_list)
		return joint_trajectory

	def move_arm_cb(self, msg):
		T = convert_from_trans_message(msg)
		self.mutex.acquire()
		q_start = self.q_from_joint_state(self.joint_state)
		print "Solving IK"
		q_goal = self.IK(T)
		if len(q_goal)==0:
			print "IK failed, aborting"
			self.mutex.release()
			return
		print "IK solved, planning"
		trajectory = self.project_plan(numpy.array(q_start), q_goal, self.q_min, self.q_max)
		if not trajectory.points:
			print "Motion plan failed, aborting"
		else:
			print "Trajectory received with " + str(len(trajectory.points)) + " points"
			self.execute(trajectory)
		self.mutex.release()
		
	def joint_states_callback(self, joint_state):
		self.mutex.acquire()
		self.joint_state = joint_state
		self.mutex.release()

	def execute(self, joint_trajectory):
		self.pub_trajectory.publish(joint_trajectory)

if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_arm', anonymous=True)
	ma = MoveArm()
	rospy.spin()