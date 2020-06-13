
#ifndef FLIP_TRAJECTORY_H
#define FLIP_TRAJECTORY_H

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

class fliptraj
{
private:
  
  ros::NodeHandle nh_;

  ros::Publisher controllertype_;

  ros::ServiceServer trajtriggerServ_;

  ros::Publisher flatreferencePub_;
  ros::Subscriber mavposeSub_;
  ros::Subscriber mavtwistSub_;

  ros::Timer refloop_timer_;
  ros::Time start_time_, curr_time_;

  Eigen::Vector3d p_targ, v_targ, a_targ, w_targ;

  double trigger_time_;
  double take_off_height, curr_height, curr_y;

  double pitch_angle;

  double init_vel, curr_vel;

  double energy;

  double r;

  double Tc;

  double m;
  
  double g;


  int motion_selector_;

  Eigen::Vector3d v_mav_;

  bool publish_data;
  
  std::vector<Eigen::Vector3d> position_;
  std::vector<Eigen::Vector3d> velocity_;

  std::vector<float> T;


public:
  fliptraj(const ros::NodeHandle& nh);

  void updateReference();
  void pubflatvelocity();
  void pubflatrefState();
  void typepublish();
  void refCallback(const ros::TimerEvent& event);
  bool triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void mavposeCallback(const geometry_msgs::PoseStamped&);
  void flip_traj_generate();
  void mavtwistCallback(const geometry_msgs::TwistStamped& msg);


  };


#endif
