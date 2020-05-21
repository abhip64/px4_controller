#include "trajectory_publisher/flip_trajectory.h"

#include <iostream>

using namespace std;
using namespace Eigen;

fliptraj::fliptraj(const ros::NodeHandle& nh) :
  nh_(nh),
  motion_selector_(0),publish_data(0),pitch_angle(0){

//Publisher
  flatreferencePub_ = nh_.advertise<control_msgs::FlatTarget>("reference/flatsetpoint", 10);
  controllertype_   = nh_.advertise<std_msgs::Int8>("/trajectory/controller_type",10);

  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 10, &fliptraj::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &fliptraj::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());


//Timers
  refloop_timer_  = nh_.createTimer(ros::Duration(0.01), &fliptraj::refCallback, this);

//Services
  trajtriggerServ_ = nh_.advertiseService("start", &fliptraj::triggerCallback, this);

  //nh_.getParam("/take_off_height", take_off_height);
  m = 1;
  
  g = 9.8;

  take_off_height = 10.0;

  init_vel = 5.0;

  curr_vel = init_vel;

  Tc = 2*m*g; 

  r = (curr_vel*curr_vel)/(Tc - g*cos(pitch_angle));

  energy = curr_vel*curr_vel/2.0;

//Variable Definitions
  position_.resize(2);
  position_.at(0) << 0.0, 0.0, take_off_height;
  position_.at(1) << 5.0, 0.0, take_off_height;

  eth_set_pos(position_.at(0),position_.at(1));

  velocity_.resize(2);
  velocity_.at(0) << 0.0, 0.0, 0.0;
  velocity_.at(1) << init_vel, 0.0, 0.0;
  eth_set_vel(velocity_.at(0), velocity_.at(1));

  T.resize(2);
  T.at(0) = 5;
  T.at(1) = 10;
    
  p_targ = position_.at(0);
  v_targ = velocity_.at(0);

  T.at(0) = eth_trajectory_init();

}

void fliptraj::updateReference() {
  curr_time_ = ros::Time::now();
  trigger_time_ = (curr_time_ - start_time_).toSec();

  if((trigger_time_ < T.at(0)))
  {
  p_targ = eth_trajectory_pos(trigger_time_);
  v_targ = eth_trajectory_vel(trigger_time_);
  w_targ = eth_trajectory_angvel(trigger_time_);
  a_targ = eth_trajectory_acc(trigger_time_);
  }
  else if(trigger_time_ < (T.at(0) + T.at(1)))
  {
  motion_selector_ = 3;
  this->flip_traj_generate();
  }
  else if(trigger_time_ > (T.at(0) + T.at(1) + 1))
  {
  motion_selector_ = 2;
  }
}


void fliptraj::pubflatrefState(){
  control_msgs::FlatTarget msg;

  msg.header.stamp = ros::Time::now();
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

void fliptraj::pubflatvelocity()
{
  control_msgs::FlatTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.position.x = 0;
  msg.position.y = 0;
  msg.position.z = 0;
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  msg.ang_vel.x = 0;
  msg.ang_vel.y = 0;
  msg.ang_vel.z = 0;

//  std::cout<<v_targ(0)<<"\n";

  typepublish();

  flatreferencePub_.publish(msg);
}

void fliptraj::typepublish()
{
  std_msgs::Int8 cont_type;

  cont_type.data = motion_selector_;

  controllertype_.publish(cont_type);
}

void fliptraj::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  if(publish_data)
  {
    updateReference();

    if(motion_selector_ == 0)
      pubflatrefState();
  	else if(motion_selector_ == 3)
  	  pubflatvelocity();
    else if(motion_selector_ == 2)
      typepublish();
  }
}

void fliptraj::flip_traj_generate()
{
	pitch_angle += (curr_vel*0.01)/r;

  curr_vel = v_mav_.norm();

  //std::cout<<curr_vel<<"  "<<(curr_vel*curr_vel)/2 + g*(curr_height - take_off_height)<<"\n";

	curr_vel = pow(2*std::max(0.0,(energy - g*(curr_height - take_off_height))),0.5);


	v_targ << curr_vel*cos(pitch_angle), 0, curr_vel*sin(pitch_angle);

	a_targ << Tc*sin(pitch_angle), 0, Tc*cos(pitch_angle) - g;

	r = (curr_vel*curr_vel)/(Tc - g*cos(pitch_angle));
}


bool fliptraj::triggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  
  publish_data = req.data;
  
  start_time_ = ros::Time::now(); 
  res.success = publish_data;

  if(publish_data)
    res.message = "Trajectory triggered";
  else
    res.message = "Trajectory stopped publishing";
}

void fliptraj::mavposeCallback(const geometry_msgs::PoseStamped& msg){

  curr_height = msg.pose.position.z;

}

void fliptraj::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {

  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;

}