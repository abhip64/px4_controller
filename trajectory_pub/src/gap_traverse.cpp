//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/gap_traverse.h"

#include <iostream>

using namespace std;

//Roll Angle is considered to be the slit angle
gap_trajectory::gap_trajectory(float T, float roll, float pitch, float yaw) :
  dt_(0.1),type_(2){

  T_ = T;

  l = 0.3;
  d = 0.3;

  gap_roll  = roll*ToRad;
  gap_pitch = pitch*ToRad;
  gap_yaw   = yaw*ToRad;

  R << cos(gap_pitch)*cos(gap_yaw), sin(gap_roll)*sin(gap_pitch)*cos(gap_yaw)-cos(gap_roll)*sin(gap_yaw), cos(gap_roll)*sin(gap_pitch)*cos(gap_yaw)+sin(gap_roll)*sin(gap_yaw),
       cos(gap_pitch)*sin(gap_yaw), sin(gap_roll)*sin(gap_pitch)*sin(gap_yaw)+cos(gap_roll)*cos(gap_yaw), cos(gap_roll)*sin(gap_pitch)*sin(gap_yaw)-sin(gap_roll)*cos(gap_yaw),
       -sin(gap_pitch),             sin(gap_roll)*cos(gap_pitch),                                         cos(gap_roll)*cos(gap_pitch); 

  pos_mid << 10, 0, 10;

  pos_launch << -l, -d, 0;

  Eigen::Vector3d g;
  
  g << 0, 0, -9.81;

  g_plane = R.transpose()*g;

  g_plane[2] = 0;

  vel_launch = -(pos_launch/T_) - (0.5*g_plane*T_);



};

gap_trajectory::~gap_trajectory() {}

int gap_trajectory::get_type()
{
  return type_;
}

Eigen::Vector3d gap_trajectory::getFinalPos()
{
  return pos_mid + R*pos_launch;
}

Eigen::Vector3d gap_trajectory::getFinalVel()
{
  return R*vel_launch;
}

Eigen::Vector3d gap_trajectory::getPosition(double time){

  Eigen::Vector3d position;
  position = pos_launch + vel_launch*time + 0.5*g_plane*pow(time,2);

  position = pos_mid + R*position;

  return position;
}

Eigen::Vector3d gap_trajectory::getVelocity(double time){

  Eigen::Vector3d velocity;
  velocity = vel_launch + g_plane*time;

  velocity = R*velocity;

  return velocity;
}

Eigen::Vector3d gap_trajectory::getAcceleration(double time){

  Eigen::Vector3d acceleration;
  acceleration << 0,
                  0,
                  0;

  return acceleration;
}

Eigen::Vector3d gap_trajectory::getJerk(double time){
return Eigen::Vector3d(0,0,0);

}

Eigen::Vector3d gap_trajectory::getW(double time)
{
return Eigen::Vector3d(0,0,0);

}

float gap_trajectory::getslit_ang(double time)
{

if(time < T_/2)
return gap_roll;
else 
return 0.0;

}

nav_msgs::Path gap_trajectory::getSegment(){

  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for(double t = 0 ; t < this->getDuration() ; t+=this->getsamplingTime()){
    targetPosition = this->getPosition(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped gap_trajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);

  return encode_msg;
}