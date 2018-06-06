# Project 3 | Robot Arms - Fordward Kinematics

To test the model:
```
$ roslaunch robot_sim kuka_lwr_test.launch
$ rosrun robot_sim position_command.py
```

To test the project:
```
$ roslaunch robot_sim kuka_lwr.launch
```

## Description

The project implements the forward kinematics for a robot arm defined in a URDF file and running in a ROS environment.

The setup contains a "simulated" robot that _continuously publishes its own joint values_. You can check that the robot is indeed publishing its joint values by using the `rostopic echo /joint_states` command. However, that is not enough for the robot to be correctly displayed: a forward kinematics module must use the joint values to compute the transforms from the world coordinate frame to each link of the robot.

The `ForwardsKinematics` class subscribes to the topic `joint_states` and publishes transforms to `tf`. It also loads a URDF description of the robot from the ROS parameter server.

### The 'compute_transforms' function

This is the function that performs the main forward kinematics computation. It accepts as parameters all the information needed about the joints and links of the robot, as well as the current values of all the joints, and must compute and return the transforms from the world frame to all the links, ready to be published through tf.

Parameters are as follows:

- _link_names_: a list with all the names of the robot's links, ordered from proximal to distal. These are also the names of the link's respective coordinate frame. In other words, the transform from the world to link _i_ should be published with _world_link_ as the parent frame and _link_names[i]_ as the child frame.    
- _joints_: a list of all the joints of the robot, in the same order as the links listed above. Each entry in this list is an object which contains the following fields:
- _joint.origin.xyz_: the translation from the frame of the previous joint to this one
- _joint.origin.rpy_: the rotation from the frame of the previous joint to this one, in ROLL-PITCH-YAW XYZ convention
- _joint.type_: either `fixed` or `revolute`. A fixed joint does not move; it is meant to contain a static transform.
- _joint.name_: the name of the current joint in the robot description
- _joint.axis_: (only if type is 'revolute') the axis of rotation of the joint
joint_values contains information about the current joint values in the robot. It contains information about all the joints, and the ordering can vary, so we must find the relevant value  for a particular joint you are considering. We can use the following fields:
- _joint_values.name_: a list of the names of all the joints in the robot;
- _joint_values.position_: a list of the current values of all the joints in the robot, in the same order as the names in the list above. To find the value of the joint we care about, we must find its name in the name list, then take the value found at the same index in the position list.
The function must return one tf message. The transforms field of this message must list all the transforms from the world coordinate frame to the links of the robot. In other words, when you are done, all_transforms.transforms must contain a list in which you must place all the transforms from the world_link coordinate frame to each of the coordinate frames listed in link_names. You can use the convert_to_message function (defined above) for a convenient way to create a tf message from a transformation matrix.

## Resources and Hints

It will help to get familiar with  the URDF documentation. In particular, the documentation for the URDF Joint element will be very helpful in understanding the nature of the joint object that is being passed to the `compute_transforms` function, and what you must do with the data in each joint object.

![ROS Graph](../utils/kuka_lwr/resources/rosgraph.png)
