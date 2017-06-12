#include "robot_sim/trajectory_executer.h"

namespace robot_sim {

bool TrajectoryExecuter::init()
{
  max_velocity_ = 1.0;
  trajectory_sub_ = root_nh_.subscribe("joint_trajectory", 10, &TrajectoryExecuter::callback, this);
  new_trajectory_ = false;
  executing_ = false;
  current_vels_.resize(robot_->getNumJoints(), 0.0);
  timer_ = root_nh_.createTimer(ros::Duration(0.01), &TrajectoryExecuter::timerCallback, this);
  return true;
}

void TrajectoryExecuter::callback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  current_trajectory_ = *msg;
  new_trajectory_ = true;
  executing_ = true;
}

void TrajectoryExecuter::stopRobot()
{
  for (size_t i=0; i<robot_->getNumJoints(); i++) {
    current_vels_[i] = 0.0;
  }
  robot_->setVelocities(current_vels_);
}

void TrajectoryExecuter::startNewSegment()
{
  std::vector<double> current_vals = robot_->getJointValues();
  std::vector<double> target_vals = current_trajectory_.points[current_point_].positions;
  double longest_distance = 0.0;
  for (size_t i=0; i<robot_->getNumJoints(); i++) {
    longest_distance = std::max(longest_distance, fabs(current_vals[i]-target_vals[i]));
  }
  if (longest_distance == 0.0) {
    stopRobot();
    return;
  }
  double trajectory_time = longest_distance / max_velocity_;
  for (size_t i=0; i<robot_->getNumJoints(); i++) {
    double dist = target_vals[i] - current_vals[i];
    current_vels_[i] = dist / trajectory_time;
  }
  robot_->setVelocities(current_vels_);
}

bool TrajectoryExecuter::targetReached()
{
  double EPS = 1.0e-2;
  std::vector<double> current_vals = robot_->getJointValues();
  std::vector<double> target_vals = current_trajectory_.points[current_point_].positions;
  bool reached = true;
  for (size_t i=0; i<robot_->getNumJoints(); i++) {
    if (fabs(current_vals[i] - target_vals[i]) > EPS) {
      reached = false;
      if ( ( current_vels_[i]>0.0 && current_vals[i] > target_vals[i] ) || 
	   ( current_vels_[i]<0.0 && current_vals[i] < target_vals[i] ) ) {
	ROS_ERROR("Joint overshoot in trajectory executer");
	return true;
      }
    }
  }
  return reached;
}

void TrajectoryExecuter::timerCallback(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (!executing_) return;
  if (new_trajectory_) {
    new_trajectory_ = false;
    if (current_trajectory_.points.empty()) {
      stopRobot();
      executing_ = false;
      return;
    } else {
      current_point_ = 0;
      startNewSegment();
      executing_ = true;
      return;
    }
  }
  if (targetReached()) {
    current_point_++;
    if (current_point_ >= current_trajectory_.points.size()) {
      stopRobot();
      executing_ = false;
      return;
    }
    else {
      startNewSegment();
    }
  }
}

}
