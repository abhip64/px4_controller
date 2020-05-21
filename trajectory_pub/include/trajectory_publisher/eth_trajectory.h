
#ifndef ETH_TRAJECTORY_H
#define ETH_TRAJECTORY_H

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Path.h>


void eth_set_pos(Eigen::Vector3d, Eigen::Vector3d);

void eth_set_vel(Eigen::Vector3d, Eigen::Vector3d);

void get_mid_pos_vel();

double eth_trajectory_init();

Eigen::Vector3d calc_inter_pos(Eigen::Vector3d);

Eigen::Vector3d eth_trajectory_pos(double time);

Eigen::Vector3d eth_trajectory_vel(double time);

Eigen::Vector3d eth_trajectory_acc(double time);

Eigen::Vector3d eth_trajectory_jerk(double time);

Eigen::Vector3d eth_trajectory_angvel(double time);

nav_msgs::Path getSegment();

geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d, Eigen::Vector4d);

#endif