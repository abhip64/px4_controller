//
// Created by jalim on 11.10.18.
//

#include "trajectory_publisher/polynomialtrajectory.h"

#include <iostream>

polynomialtrajectory::polynomialtrajectory(float T) :
  dt_(0.1),type_(1){

  T_ = T;


};

polynomialtrajectory::~polynomialtrajectory() {}

int polynomialtrajectory::get_type()
{
  return type_;
}

void polynomialtrajectory::generatePrimitives(Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d jerk, Eigen::Vector3d pos_, Eigen::Vector3d vel_, Eigen::Vector3d acc_, Eigen::Vector3d jerk_)
{

  Eigen::MatrixXd A(8,8);
  Eigen::VectorXd B(8);

  A << 1, 0, 0,        0,          0,           0,           0,            0,
       0, 1, 0,        0,          0,           0,           0,            0,
       0, 0, 2,        0,          0,           0,           0,            0,
       0, 0, 0,        6,          0,           0,           0,            0,
       1, T_, pow(T_,2), pow(T_,3),   pow(T_,4),    pow(T_,5),    pow(T_,6),     pow(T_,7),
       0, 1, 2*T_,      3*pow(T_,2), 4*pow(T_,3),  5*pow(T_,4),  6*pow(T_,5),   7*pow(T_,6),
       0, 0, 2,        6*T_,        12*pow(T_,2), 20*pow(T_,3), 30*pow(T_,4),  42*pow(T_,5),
       0, 0, 0,        6,          24*T_,        60*pow(T_,2), 120*pow(T_,3), 210*pow(T_,4);

  B << pos(0) ,
       vel(0) ,
       acc(0) ,
       jerk(0),
       pos_(0)  ,
       vel_(0)  ,
       acc_(0)  ,
       jerk_(0);     

  c_x = A.colPivHouseholderQr().solve(B);

  B << pos(1) ,
       vel(1) ,
       acc(1) ,
       jerk(1),
       pos_(1)  ,
       vel_(1)  ,
       acc_(1)  ,
       jerk_(1); 

  c_y = A.colPivHouseholderQr().solve(B);

  B << pos(2) ,
       vel(2) ,
       acc(2) ,
       jerk(2),
       pos_(2)  ,
       vel_(2)  ,
       acc_(2)  ,
       jerk_(2);         

  c_z = A.colPivHouseholderQr().solve(B);

}

Eigen::VectorXd polynomialtrajectory::getCoefficients(int dim){

  switch(dim) {
    case 0 :
      return c_x;
    case 1 :
      return c_y;
    case 2 :
      return c_z;
  }
}

Eigen::Vector3d polynomialtrajectory::getPosition(double time){

  Eigen::Vector3d position;
  position << c_x(0) + c_x(1) * time + c_x(2) * pow(time, 2) + c_x(3) * pow(time, 3) + c_x(4) * pow(time, 4) + c_x(5) * pow(time, 5) + c_x(6) * pow(time, 6) + c_x(7) * pow(time, 7),
              c_y(0) + c_y(1) * time + c_y(2) * pow(time, 2) + c_y(3) * pow(time, 3) + c_y(4) * pow(time, 4) + c_y(5) * pow(time, 5) + c_y(6) * pow(time, 6) + c_y(7) * pow(time, 7),
              c_z(0) + c_z(1) * time + c_z(2) * pow(time, 2) + c_z(3) * pow(time, 3) + c_z(4) * pow(time, 4) + c_z(5) * pow(time, 5) + c_z(6) * pow(time, 6) + c_z(7) * pow(time, 7);

  return position;
}

Eigen::Vector3d polynomialtrajectory::getVelocity(double time){

  Eigen::Vector3d velocity;
  velocity << c_x(1) + c_x(2) * time * 2 + c_x(3) * pow(time, 2) * 3 + c_x(4) * pow(time, 3) * 4 + c_x(5) * pow(time, 4) * 5 + c_x(6) * pow(time, 5) * 6 + c_x(7) * pow(time, 6) * 7,
              c_y(1) + c_y(2) * time * 2 + c_y(3) * pow(time, 2) * 3 + c_y(4) * pow(time, 3) * 4 + c_y(5) * pow(time, 4) * 5 + c_y(6) * pow(time, 5) * 6 + c_y(7) * pow(time, 6) * 7,
              c_z(1) + c_z(2) * time * 2 + c_z(3) * pow(time, 2) * 3 + c_z(4) * pow(time, 3) * 4 + c_z(5) * pow(time, 4) * 5 + c_z(6) * pow(time, 5) * 6 + c_z(7) * pow(time, 6) * 7;

  return velocity;
}

Eigen::Vector3d polynomialtrajectory::getAcceleration(double time){

  Eigen::Vector3d acceleration;
  acceleration << c_x(2) * 2 + c_x(3) * time * 6 + c_x(4) * pow(time,2) * 12 + c_x(5) * pow(time,3) * 20 + c_x(6) * pow(time,4) * 30 + c_x(7) * pow(time,5) * 42,
                  c_y(2) * 2 + c_y(3) * time * 6 + c_y(4) * pow(time,2) * 12 + c_y(5) * pow(time,3) * 20 + c_y(6) * pow(time,4) * 30 + c_y(7) * pow(time,5) * 42,
                  c_z(2) * 2 + c_z(3) * time * 6 + c_z(4) * pow(time,2) * 12 + c_z(5) * pow(time,3) * 20 + c_z(6) * pow(time,4) * 30 + c_z(7) * pow(time,5) * 42;

  return acceleration;
}

Eigen::Vector3d polynomialtrajectory::getJerk(double time){

  Eigen::Vector3d jerk;
  jerk <<  c_x(3) * 6 + c_x(4) * pow(time,1) * 24 + c_x(5) * pow(time,2) * 60 + c_x(6) * pow(time,3) * 120 + c_x(7) * pow(time,4) * 210,
           c_y(3) * 6 + c_y(4) * pow(time,1) * 24 + c_y(5) * pow(time,2) * 60 + c_y(6) * pow(time,3) * 120 + c_y(7) * pow(time,4) * 210,
           c_z(3) * 6 + c_z(4) * pow(time,1) * 24 + c_z(5) * pow(time,2) * 60 + c_z(6) * pow(time,3) * 120 + c_z(7) * pow(time,4) * 210;

  return jerk;
}

Eigen::Vector3d polynomialtrajectory::getW(double time)
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, -9.8;
  
  acc = getAcceleration(time) - g;
  jerk = getJerk(time);
  
  double u = acc.norm();

  zb = acc/u;

////////////NEEDS UPDATE//////////////////////// 
  xc << 1,0,0;
///////////////////////////////////////////////

  yb = zb.cross(xc) / (zb.cross(xc)).norm();
  xb = yb.cross(zb) / (yb.cross(zb)).norm();

  h = (m/u)*(jerk - (zb.dot(jerk))*zb);

  w(0) = -h.dot(yb);

  w(1) = h.dot(xb);

  w(2) = 0;

  return w;

}

nav_msgs::Path polynomialtrajectory::getSegment(){

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

float polynomialtrajectory::getslit_ang(double time)
{

return 0;

}

geometry_msgs::PoseStamped polynomialtrajectory::vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
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