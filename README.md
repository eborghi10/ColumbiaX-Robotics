# CSMM.103x - ColumbiaX | Robotics | edX

1) **project1**: publish and subscribe custom messages.

2) **tf2_examples**: basic example using the package _tf2_ for coordinate system's transformations.

3) **project2_solution**: program that uses tf to describe the poses between a robot, its camera and an object.

4) **marker_publisher**: used in _project2_solution_ to visualize the transformations. It's invoked by _project2_solution_.

5) **project3**: packages that analyze the forward dynamics of a Kuka LBR iiwa arm.

6) **project4/cartesian_control**: also with the package `robot_sim` from _project3_, uses the differential kinematics of the Kuka LBR iiwa robot arm.

7) **project5**: In order for the robot end-effector to reach a desired pose without collisions with any obstacles present in its environment it's necessary to implement motion planning. In this project is used a Rapidly-exploring Random Tree (RRT) motion planner for the same 7-jointed robot arm of the previous lectures.