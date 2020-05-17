//
// Created by jalim on 11.10.18.
//

#ifndef TRAJECTORY_PUBLISHER_GAP_TRAVERSE_H
#define TRAJECTORY_PUBLISHER_GAP_TRAVERSE_H

#include "trajectory_publisher/trajectory.h"

#define ToRad 0.0174532925

class gap_trajectory : public trajectory {
private:

  double dt_; //Sampling time
  double T_;
  int type_;
  Eigen::Vector3d pos_mid;
  Eigen::Vector3d pos_launch;
  Eigen::Vector3d vel_launch;

  Eigen::Vector3d g_plane;

  float l;   //X Axis
  float d;   //Y Axis

  float gap_roll;
  float gap_pitch;
  float gap_yaw;

  Eigen::Matrix3d R;

public:
  gap_trajectory(float T, float roll, float pitch, float yaw);
  virtual ~gap_trajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega){}
  void generatePrimitives(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d){}
  Eigen::Vector3d getFinalPos();
  Eigen::Vector3d getFinalVel();
  Eigen::Vector3d getPosition(double time);
  Eigen::Vector3d getVelocity(double time);
  Eigen::Vector3d getAcceleration(double time);
  Eigen::Vector3d getJerk(double time);
  Eigen::Vector3d getW(double time);
  float getslit_ang(double time);
  int get_type();

  double getsamplingTime(){return dt_;};
  double getDuration(){ return T_;};
  nav_msgs::Path getSegment();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation);
};


#endif //TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
