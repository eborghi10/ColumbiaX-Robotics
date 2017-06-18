#include "robot_sim_prj5/velocity_controller.h"

namespace robot_sim_prj5 {

bool VelocityController::init()
{
  velocity_sub_ = root_nh_.subscribe("joint_velocities", 10, &VelocityController::callback, this);
  return true;
}

void VelocityController::callback(const sensor_msgs::JointState::ConstPtr &msg)
{
  if (msg->velocity.size() != robot_->getNumJoints())
  {
    ROS_ERROR_STREAM("Received joint velocities (" << msg->velocity.size() <<
		     ") do not match robot(" << robot_->getNumJoints() << 
		     ")");
    return;
  }
  robot_->setVelocities(msg->velocity);
}

}
