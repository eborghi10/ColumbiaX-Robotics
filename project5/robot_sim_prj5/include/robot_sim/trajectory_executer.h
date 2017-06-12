#pragma once

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "robot_sim/robot.h"

namespace robot_sim {

class TrajectoryExecuter
{
 public:
 TrajectoryExecuter(boost::shared_ptr<Robot> robot) : 
  root_nh_(""), robot_(robot)
  {}

  bool init();

 private:
  ros::NodeHandle root_nh_;
  boost::shared_ptr<Robot> robot_;
  ros::Subscriber trajectory_sub_;
  mutable boost::mutex mutex_;
  trajectory_msgs::JointTrajectory current_trajectory_;
  size_t current_point_;
  bool new_trajectory_;
  bool executing_;
  //! Max velocity in rad/s
  double max_velocity_;
  std::vector<double> current_vels_;
  ros::Timer timer_;

  void callback(const trajectory_msgs::JointTrajectory::ConstPtr&);
  void timerCallback(const ros::TimerEvent&);

  void startNewSegment();
  bool targetReached();
  void stopRobot();
};

} //namespace robot_sim
