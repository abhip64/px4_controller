//
// Created by jalim on 05.05.18.
//

#include "trajectory_publisher/trajectoryPublisher.h"

#include <iostream>

using namespace std;
using namespace Eigen;
trajectoryPublisher::trajectoryPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private) :
  nh_(nh),
  nh_private_(nh_private),
  motion_selector_(0),publish_data(0) {

  trajectoryPub_ = nh_.advertise<nav_msgs::Path>("/trajectory_publisher/trajectory", 10);
  referencePub_ = nh_.advertise<geometry_msgs::TwistStamped>("reference/setpoint", 10);
  flatreferencePub_ = nh_.advertise<controller_msgs::FlatTarget>("reference/flatsetpoint", 10);
  rawreferencePub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  motionselectorSub_ = nh_.subscribe("/trajectory_publisher/motionselector", 10, &trajectoryPublisher::motionselectorCallback, this,ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 10, &trajectoryPublisher::mavposeCallback, this,ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity", 10, &trajectoryPublisher::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());

  trajloop_timer_ = nh_.createTimer(ros::Duration(0.1), &trajectoryPublisher::loopCallback, this);
  refloop_timer_ = nh_.createTimer(ros::Duration(0.01), &trajectoryPublisher::refCallback, this);

  trajtriggerServ_ = nh_.advertiseService("start", &trajectoryPublisher::triggerCallback, this);

  nh_.param<double>("/trajectory_publisher/initpos_x", init_pos_x_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_y", init_pos_y_, 0.0);
  nh_.param<double>("/trajectory_publisher/initpos_z", init_pos_z_, 10.0);
  nh_.param<double>("/trajectory_publisher/updaterate", controlUpdate_dt_, 0.01);
  nh_.param<double>("/trajectory_publisher/horizon", primitive_duration_, 1.0);
  nh_.param<double>("/trajectory_publisher/maxjerk", max_jerk_, 10.0);
  nh_.param<double>("/trajectory_publisher/shape_omega", shape_omega_, 1.5);
  nh_.param<int>("/trajectory_publisher/trajectory_type", trajectory_type_, 0);
  nh_.param<int>("/trajectory_publisher/number_of_primitives", num_primitives_, 2);
  nh_.param<int>("/trajectory_publisher/reference_type", pubreference_type_, 2);


  position_.resize(num_primitives_);
  velocity_.resize(num_primitives_ );
  accel_.resize(num_primitives_ );
  jerk_.resize(num_primitives_ );
  T.resize(num_primitives_);

  if(trajectory_type_ == 0){//Polynomial Trajectory

      T.at(0) = 5;
      T.at(1) = 1;
     // T.at(2) = 3;

      //gap_traj = std::make_shared<gap_trajectory>(T.at(num_primitives_),30.0,0.0); 
    
    for(int i = 0;  i < num_primitives_ - 1; i++){

      motionPrimitives_.emplace_back(std::make_shared<polynomialtrajectory>(T.at(i)));
      primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset" + std::to_string(i), 1));
    }
      motionPrimitives_.emplace_back(std::make_shared<gap_trajectory>(T.at(num_primitives_-1),-45.0,0.0,0.0));
      primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset" + std::to_string(num_primitives_-1), 1));


      position_.at(0) << 0.0, 0.0, 10.0;
      position_.at(1) = motionPrimitives_.at(num_primitives_-1)->getFinalPos();
      //position_.at(1) << 5.0, 0.0, 12.0;
     // position_.at(2) << 10.0, 0.0, 10.0;

      velocity_.at(0) << 0.0, 0.0, 0.0;
      velocity_.at(1) = motionPrimitives_.at(num_primitives_-1)->getFinalVel();
      //velocity_.at(1) << 5.0, 0.0, 5.0;
     // velocity_.at(2) << 0.0, 0.0, 0.0;

      accel_.at(0)    << 0.0, 0.0, 0.0;
      accel_.at(1)    << 0.0, 0.0, 0.0;
     //accel_.at(2)    << 0.0, 0.0, 0.0;

      jerk_.at(0)     << 0.0, 0.0, 0.0;
      jerk_.at(1)     << 0.0, 0.0, 0.0;
     // jerk_.at(2)     << 0.0, 0.0, 0.0;

  }
  else {//Shape trajectories

    num_primitives_ = 1;
    motionPrimitives_.emplace_back(std::make_shared<shapetrajectory>(trajectory_type_));
    primitivePub_.push_back(nh_.advertise<nav_msgs::Path>("/trajectory_publisher/primitiveset", 1));
  }

  p_targ << init_pos_x_, init_pos_y_, init_pos_z_;
  v_targ << 0.0, 0.0, 0.0;
  shape_origin_ << init_pos_x_, init_pos_y_, init_pos_z_;
  shape_axis_ << 0.0, 0.0, 1.0;
  motion_selector_ = 0;

  initializePrimitives(trajectory_type_);

}

void trajectoryPublisher::updateReference() {
  curr_time_ = ros::Time::now();
  trigger_time_ = (curr_time_ - start_time_).toSec();

  if((trigger_time_ >  T.at(motion_selector_)) && (motion_selector_ < num_primitives_-1))
   { 
    ++motion_selector_;
    start_time_ = ros::Time::now();
    trigger_time_ = 0;
  }
  //std::cout<<motion_selector_<<"\n";
  if((trigger_time_ < T.at(motion_selector_)))
  {
  p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
  v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);
  w_targ = motionPrimitives_.at(motion_selector_)->getW(trigger_time_);
  slit_angle_targ = motionPrimitives_.at(motion_selector_)->getslit_ang(trigger_time_);
  //cout<<w_targ<<"\n";
  if(pubreference_type_!=0) a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
  }

}

void trajectoryPublisher::initializePrimitives(int type){
  if(type == 0){

    for(int i = 0; i < num_primitives_ - 1; i++ ) 
    motionPrimitives_.at(i)->generatePrimitives(position_.at(i),velocity_.at(i),accel_.at(i), jerk_.at(i), position_.at(i+1),velocity_.at(i+1),accel_.at(i+1), jerk_.at(i+1));
  }
  else {
    for(int i = 0; i < motionPrimitives_.size(); i++ ) motionPrimitives_.at(i)->initPrimitives(shape_origin_, shape_axis_, shape_omega_);
    //TODO: Pass in parameters for primitive trajectories
    
  }
}

void trajectoryPublisher::updatePrimitives(){
  //for(int i = 0; i < motionPrimitives_.size() ; i++ ) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_);
}

void trajectoryPublisher::pubrefTrajectory(int selector){
  //Publish current trajectory the publisher is publishing
  refTrajectory_ = motionPrimitives_.at(selector)->getSegment();
  refTrajectory_.header.stamp = ros::Time::now();
  refTrajectory_.header.frame_id = "map";
  trajectoryPub_.publish(refTrajectory_);

}

void trajectoryPublisher::pubprimitiveTrajectory(){

  for(int i = 0; i < motionPrimitives_.size(); i++ ){
    primTrajectory_ = motionPrimitives_.at(i)->getSegment();
    primTrajectory_.header.stamp = ros::Time::now();
    primTrajectory_.header.frame_id = "map";
    primitivePub_.at(i).publish(primTrajectory_);
  }

}

void trajectoryPublisher::pubrefState(){
  geometry_msgs::TwistStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.twist.angular.x = p_targ(0);
  msg.twist.angular.y = p_targ(1);
  msg.twist.angular.z = p_targ(2);
  msg.twist.linear.x = v_targ(0);
  msg.twist.linear.y = v_targ(1);
  msg.twist.linear.z = v_targ(2);
  referencePub_.publish(msg);
}

void trajectoryPublisher::pubflatrefState(){
  controller_msgs::FlatTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = motionPrimitives_.at(motion_selector_)->get_type();
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
  msg.slit_ang = slit_angle_targ;
  flatreferencePub_.publish(msg);
}

void trajectoryPublisher::pubrefSetpointRaw(){
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = 64;
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  rawreferencePub_.publish(msg);
}


void trajectoryPublisher::loopCallback(const ros::TimerEvent& event){
  //Slow Loop publishing trajectory information
  if(publish_data)
  {
  pubrefTrajectory(motion_selector_);
  pubprimitiveTrajectory();
  }

}

void trajectoryPublisher::refCallback(const ros::TimerEvent& event){
  //Fast Loop publishing reference states
  if(publish_data)
  {
  updateReference();
  switch(pubreference_type_){
    case REF_TWIST :
      pubrefState();
      break;
    case REF_SETPOINTRAW :
      pubrefSetpointRaw();
      break;
    default : 
      pubflatrefState();
      break;
  }
  }
}

bool trajectoryPublisher::triggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  
  publish_data = req.data;
  
  start_time_ = ros::Time::now(); 
  res.success = publish_data;

  if(publish_data)
    res.message = "Trajectory triggered";
  else
    res.message = "Trajectory stopped publishing";
}

void trajectoryPublisher::motionselectorCallback(const std_msgs::Int32& selector_msg){

  motion_selector_ = selector_msg.data;
  updatePrimitives();
  start_time_ = ros::Time::now();

}

void trajectoryPublisher::mavposeCallback(const geometry_msgs::PoseStamped& msg){

  p_mav_(0) = msg.pose.position.x;
  p_mav_(1) = msg.pose.position.y;
  p_mav_(2) = msg.pose.position.z;
  updatePrimitives();

}

void trajectoryPublisher::mavtwistCallback(const geometry_msgs::TwistStamped& msg) {

  v_mav_(0) = msg.twist.linear.x;
  v_mav_(1) = msg.twist.linear.y;
  v_mav_(2) = msg.twist.linear.z;
  updatePrimitives();

}