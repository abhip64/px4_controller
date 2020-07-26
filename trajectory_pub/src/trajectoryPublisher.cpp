#include "trajectory_publisher/trajectoryPublisher.h"

#include <iostream>

using namespace std;
using namespace Eigen;

trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh) :
  nh_(nh),
  motion_selector_(0),publish_data(0){

//Publisher
  trajectoryPub_    = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/trajectory", 10);
  flatreferencePub_ = nh_.advertise<control_msgs::FlatTarget>("reference/flatsetpoint", 100);
  controllertype_   = nh_.advertise<std_msgs::Int8>("/trajectory/controller_type",100);
  anglePub_         = nh_.advertise<control_msgs::RollPitchTarget>("/trajectory/angle",100);

//Timers
  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_  = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

//Services
  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  //nh_.getParam("/take_off_height", take_off_height);
  take_off_height = 10;

//Variable Definitions
  position_.resize(2);
  position_.at(0) << 0.0, 0.0, take_off_height;
  position_.at(1) << 5.0, 5.0, 12.0;
  //position_.at(1) << 1.0, 0.0, 10.0;
  eth_set_pos(position_.at(0),position_.at(1));

  velocity_.resize(2);
  velocity_.at(0) << 0.0, 0.0, 0.0;
  velocity_.at(1) << 1.0, 0.0, 0.0;
  //velocity_.at(1) << 0.0, 0.0, 0.0;
  eth_set_vel(velocity_.at(0), velocity_.at(1));

  T.resize(2);
  T.at(1) = 5.0;
    
  p_targ = position_.at(0);
  v_targ = velocity_.at(0);

  roll_angle  =  5.0;
  pitch_angle =  0.0;

  T.at(0) = eth_trajectory_init();
  //T.at(0) = 5;
}

void trajectoryPublisher::updateReference() {
  
  trigger_time_ = (curr_time_ - start_time_).toSec();

  if((trigger_time_ < T.at(0)))
  {
  p_targ = eth_trajectory_pos(trigger_time_);
  v_targ = eth_trajectory_vel(trigger_time_);
  w_targ = eth_trajectory_angvel(trigger_time_);
  a_targ = eth_trajectory_acc(trigger_time_);
  }
  //else if(trigger_time_ < (T.at(0) + T.at(1)))
  //{
  //motion_selector_ = 1;
  //}
  else
  {
  motion_selector_ = 2;
  }

}


void trajectoryPublisher::pubrefTrajectory(int selector){
  //Publish current trajectory the publisher is publishing
  refTrajectory_ = getSegment();
  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = "map";
  trajectoryPub_.publish(refTrajectory_);

}


void trajectoryPublisher::pubflatrefState(){
  control_msgs::FlatTarget msg;

  msg.header.stamp = curr_time_;
  msg.header.frame_id = "map";
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  msg.ang_vel.x = w_targ(0);
  msg.ang_vel.y = w_targ(1);
  msg.ang_vel.z = w_targ(2);

  typepublish();
  flatreferencePub_.publish(msg);
}



void trajectoryPublisher::loopCallback(const ros::TimerEvent& event){
  //Slow Loop publishing trajectory information
  if(publish_data)
  {
  pubrefTrajectory(motion_selector_);

  }
}

void trajectoryPublisher::anglerefState()
{
  control_msgs::RollPitchTarget ang_sp;

  double dt = ((curr_time_ - start_time_).toSec() - T.at(0));

  ang_sp.header.stamp = ros::Time::now();
  ang_sp.header.frame_id = "map";

  //if(dt<(T.at(1)/2))
  //  ang_sp.roll = 2*roll_angle*dt/(T.at(1));
  //else
    ang_sp.roll = roll_angle;
  
  ang_sp.pitch = pitch_angle;

  typepublish();
  anglePub_.publish(ang_sp);

}

void trajectoryPublisher::typepublish()
{
  std_msgs::Int8 cont_type;

  cont_type.data = motion_selector_;

  controllertype_.publish(cont_type);
}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  curr_time_ = ros::Time::now();
  if(publish_data)
  {
    updateReference();

    if(motion_selector_ == 0)
      pubflatrefState();
    else if(motion_selector_ == 1)
      anglerefState();
    else if(motion_selector_ == 2)
      typepublish();
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  
  publish_data = req.data;
  
  start_time_ = ros::Time::now(); 
  res.success = publish_data;

  if(publish_data)
    res.message = "Trajectory triggered";
  else
    res.message = "Trajectory stopped publishing";
}

