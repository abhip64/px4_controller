//
// Created by jalim on 11.10.18.
//

#ifndef TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H
#define TRAJECTORY_PUBLISHER_POLYNOMIALTRAJECTORY_H

#include "trajectory_publisher/trajectory.h"


class polynomialtrajectory : public trajectory {
private:
  int N; //Degree of polynomial
  double dt_; //Sampling time
  double T_;
  int type_;
  Eigen::VectorXd c_x, c_y, c_z;

public:
  polynomialtrajectory(float T);
  virtual ~polynomialtrajectory();
  void initPrimitives(Eigen::Vector3d pos, Eigen::Vector3d axis, double omega){}
  void generatePrimitives(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d);
  Eigen::Vector3d getFinalPos(){}
  Eigen::Vector3d getFinalVel(){}
  Eigen::VectorXd getCoefficients(int dim);
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
