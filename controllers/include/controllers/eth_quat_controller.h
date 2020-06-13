
#ifndef ETH_QUAT_CONTROLLER_H
#define ETH_QUAT_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include "controllers/math_operations.h"
#include "controllers/controller_class.h"

class eth_controller: public control_class{

private:

    ros::NodeHandle nh_;

    Eigen::Vector3d Kpos_, Kvel_, D_, Kr_, Kw_, Ki_, Ai;

    double Kpos_x_, Kpos_y_, Kpos_z_;
    double Kz_i;
    double Kvel_x_, Kvel_y_, Kvel_z_;
    double Kr, Kw;
    double Dx_, Dy_, Dz_;

    Eigen::Vector3d g_;

    double norm_thrust_const_, norm_thrust_offset_;
    double max_fb_acc_;

public:

	eth_controller(const ros::NodeHandle& nh);
    
    virtual ~eth_controller();

	Eigen::Vector4d pos_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, double&, Eigen::Vector4d&, Eigen::Vector4d&);
	Eigen::Vector4d vel_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&);
    Eigen::Vector4d ang_control(double&, double, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&);
    Eigen::Vector4d acc2quaternion(const Eigen::Vector3d vector_acc, double yaw);
	Eigen::Vector4d attcontroller(const Eigen::Vector4d&, const Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector3d&, double);
    Eigen::Vector4d flip_control(Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector4d&, Eigen::Vector4d&);

};



#endif 