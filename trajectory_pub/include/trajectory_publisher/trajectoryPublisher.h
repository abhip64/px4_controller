//  May/2018, ETHZ, Jaeyoung Lim, jalim@ethz.ch

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include "control_msgs/FlatTarget.h"
#include "control_msgs/RollPitchTarget.h"
#include "trajectory_publisher/eth_trajectory.h"

using namespace std;
using namespace Eigen;

class trajectoryPublisher
{
private:
  ros::NodeHandle nh_;

  ros::Publisher trajectoryPub_;

  ros::Publisher flatreferencePub_;

  ros::Publisher controllertype_, anglePub_ ;

  ros::ServiceServer trajtriggerServ_;
  ros::Timer trajloop_timer_;
  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  nav_msgs::Path refTrajectory_;

  Eigen::Vector3d p_targ, v_targ, a_targ, w_targ;

  double trigger_time_;
  double take_off_height;
  double roll_angle, pitch_angle;

  int motion_selector_;

  bool publish_data;
  
  std::vector<Eigen::Vector3d> position_;
  std::vector<Eigen::Vector3d> velocity_;

  std::vector<float> T;


public:
  trajectoryPublisher(const ros::NodeHandle& nh);
  void updateReference();
  void pubrefTrajectory(int selector);

  void pubflatrefState();
  void typepublish();
  void anglerefState();
  void loopCallback(const ros::TimerEvent& event);
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  };


#endif
