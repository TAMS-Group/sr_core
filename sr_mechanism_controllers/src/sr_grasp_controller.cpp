/*
 * sr_grasp_controller.cpp
 *
 *  Created on: 14 Dec 2015
 *      Author: vahid
 */

#include "sr_grasp_controller.hpp"
#include "moveit_msgs/Grasp.h"
#include "ros/ros.h"

namespace controller
{

SrGraspController::SrGraspController()
{
}

SrGraspController::~SrGraspController()
{
}
bool SrGraspController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_ = n;
  if (!node_.getParam("grasp_topic_name", grasp_topic_name_))
  {
    ROS_WARN_STREAM("no grasp topic name is provided. Default name is used");
    grasp_topic_name_ = "grasp_topic_name";
  }
  grasp_message_subscriber_ = node_.subscribe<moveit_msgs::Grasp>("grasp_topic", 1, &SrGraspController::set_grasp_command, this);
  return true;
}
void SrGraspController::starting(const ros::Time &time)
{
  return;
}
void SrGraspController::update(const ros::Time &time, const ros::Duration &period)
{
  // TODO(vahid): do the stuff that needs to be done
  return;
}
void SrGraspController::stopping(const ros::Time& time)
{
  return;
}
void SrGraspController::set_grasp_command(const moveit_msgs::GraspConstPtr& msg)
{
  boost::mutex::scoped_lock lock(read_mutex_);
  {
    current_grasp_ = *msg;
  }
}
} /* namespace controller */
