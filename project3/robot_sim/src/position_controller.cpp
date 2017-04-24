#include "robot_sim/position_controller.h"

namespace robot_sim {

bool PositionController::init()
{
  //baxter_command_sub_ = root_nh_.subscribe("/robot/limb/left/joint_command", 
  //					   10, &PositionController::baxter_callback, this);
  generic_command_sub_ = root_nh_.subscribe("joint_positions", 
					    10, &PositionController::generic_callback, this);
  return true;
}
  /*
void PositionController::baxter_callback(const baxter_core_msgs::JointCommand::ConstPtr &msg)
{
  robot_->setTargetValues(msg->names, msg->command);
}
  */
void PositionController::generic_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->position.size() != robot_->getNumJoints())
  {
    ROS_ERROR_STREAM("Received joint positions (" << msg->position.size() <<
		     ") do not match robot(" << robot_->getNumJoints() << 
		     ")");
    return;
  }
  robot_->setTargetValues(msg->position);
}

bool PositionCommand::init()
{
  generic_command_sub_ = root_nh_.subscribe("joint_command", 
					    10, &PositionCommand::generic_callback, this);
  return true;
}

void PositionCommand::generic_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->position.size() != robot_->getNumJoints())
  {
    ROS_ERROR_STREAM("Received joint positions (" << msg->position.size() <<
		     ") do not match robot(" << robot_->getNumJoints() << 
		     ")");
    return;
  }
  robot_->setJointValues(msg->position);
}

}
