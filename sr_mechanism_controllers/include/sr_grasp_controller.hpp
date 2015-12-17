/*
 * sr_grasp_controller.hpp
 *
 *  Created on: 14 Dec 2015
 *      Author: vahid
 */

#ifndef SR_CORE_SR_MECHANISM_CONTROLLERS_SRC_SR_GRASP_CONTROLLER_HPP_
#define SR_CORE_SR_MECHANISM_CONTROLLERS_SRC_SR_GRASP_CONTROLLER_HPP_

#include <sr_mechanism_controllers/sr_controller.hpp>
//#include "ros_"
#include <moveit_msgs/Grasp.h>
#include "ros/ros.h"
#include <string>
#include "boost/thread.hpp"

namespace controller
{

class SrGraspController: public SrController
{
public:
  SrGraspController();
  virtual ~SrGraspController();
  bool init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n);
  virtual void starting(const ros::Time &time);
  virtual void update(const ros::Time &time, const ros::Duration &period);
  virtual void stopping(const ros::Time& time);
private:
  void set_grasp_command(const moveit_msgs::GraspConstPtr &msg);
  moveit_msgs::Grasp current_grasp_;
  ros::Subscriber grasp_message_subscriber_;
  std::string grasp_topic_name_;
  boost::mutex read_mutex_;
};
} /* namespace controller */

#endif /* SR_CORE_SR_MECHANISM_CONTROLLERS_SRC_SR_GRASP_CONTROLLER_HPP_ */
