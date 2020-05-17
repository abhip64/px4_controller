//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
//Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  fail_detec_(false),
  ctrl_enable_(true),
  landing_commanded_(false),
  feedthrough_enable_(false),
  gap_state(0),
  node_state(WAITING_FOR_HOME_POSE) {

  referenceSub_=nh_.subscribe("reference/setpoint",10, &geometricCtrl::targetCallback,this,ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 10, &geometricCtrl::flattargetCallback, this, ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ = nh_.subscribe("reference/yaw", 10, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("/command/trajectory", 10, &geometricCtrl::multiDOFJointCallback, this, ros::TransportHints().tcpNoDelay());
  mavstateSub_ = nh_.subscribe("/mavros/state", 10, &geometricCtrl::mavstateCallback, this,ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("/mavros/local_position/pose", 10, &geometricCtrl::mavposeCallback, this,ros::TransportHints().tcpNoDelay());

  imusub_ = nh_.subscribe("/mavros/imu/data", 10, &geometricCtrl::imuCallback, this,ros::TransportHints().tcpNoDelay());

  mavtwistSub_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &geometricCtrl::mavtwistCallback, this,ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService("trigger_controller", &geometricCtrl::ctrltriggerCallback, this);
  
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback, this); // Define timer for constant loop rate

  start_traj_client_ = nh_.serviceClient<std_srvs::SetBool>("start");

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 10);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 10);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("/geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 10);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  nh_private_.param<string>("mavname", mav_name_, "quad");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, MODE_BODYRATE);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.04); // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.0); // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 15.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 15.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 15.0);
  nh_private_.param<double>("Kr", Kr, 10.0);
  nh_private_.param<double>("Kw", Kw, 0.20);
  nh_private_.param<double>("Kv_x", Kvel_x_, 10);
  nh_private_.param<double>("Kv_y", Kvel_y_, 10);
  nh_private_.param<double>("Kv_z", Kvel_z_, 10);
  nh_private_.param<int>("Controller", controller, 1);  //1 for Faesseler, 2 for VK

/*  nh_private_.param<double>("Kp_x", Kpos_x_, 5);
  nh_private_.param<double>("Kp_y", Kpos_y_, 5);
  nh_private_.param<double>("Kp_z", Kpos_z_, 5);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1);
  nh_private_.param<double>("Kv_z", Kvel_z_, 1); 
*/

  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);

  targetPos_ << 0.0, 0.0, 10.0; //Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Kr_   << -Kr     , -Kr     , -Kr;
  Kw_   << -Kw     , -Kw     , -Kw;
  
  D_ << dx_, dy_, dz_;

  tau << tau_x, tau_y, tau_z;

}
geometricCtrl::~geometricCtrl() {
  //Destructor
}

void geometricCtrl::takeoff()
{

}
void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped& msg) {

  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if(reference_request_dt_ > 0) targetAcc_ = (targetVel_ - targetVel_prev_ ) / reference_request_dt_;
  else targetAcc_ = Eigen::Vector3d::Zero();

}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget& msg) {

  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);

  targetVel_ = toEigen(msg.velocity);

  if(msg.type_mask == 1) {
    targetAcc_  = toEigen(msg.acceleration);
    targetW_    = toEigen(msg.ang_vel);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    slit_ang    = 0;
    gap_state   = 0;

  } else if (msg.type_mask == 2) {

    targetAcc_  = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    slit_ang    = 0;
    gap_state   = 0;

  } 
    else if (msg.type_mask == 3)
    {
    targetAcc_  = Eigen::Vector3d::Zero();
    targetW_    = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    slit_ang = msg.slit_ang;
    gap_state   = 1;
    }

  else if (msg.type_mask == 4) {
    targetAcc_  = Eigen::Vector3d::Zero();
    targetW_    = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    slit_ang    = 0;
    gap_state   = 0;
  } 
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32& msg) {
  if(!velocity_yaw_) mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory& msg) {

  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped& msg){
  if(!received_home_pose){
      received_home_pose = true;
      home_pose_ = msg.pose;
      ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position);
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped& msg){  
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
  
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
  node_state = LANDING;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent& event){
  switch (node_state) {
  case WAITING_FOR_HOME_POSE:
      //ctrl_mode_ = 0;
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = TAKE_OFF;
      break;

  case MISSION_EXECUTION:

    if(!feedthrough_enable_ )  computeBodyRateCmd(cmdBodyRate_);
    pubReferencePose(targetPos_, q_des);
    pubRateCommands(cmdBodyRate_);
    appendPoseHistory();
    pubPoseHistory();
    break;

  case TAKE_OFF: {

    geometry_msgs::PoseStamped takeoffmsg;
    takeoffmsg.header.stamp = ros::Time::now();
    
    takeoffmsg.pose.position.x = targetPos_[0];
    takeoffmsg.pose.position.y = targetPos_[1];
    takeoffmsg.pose.position.z = targetPos_[2];

    target_pose_pub_.publish(takeoffmsg);

    //ros::Duration(10).sleep();
    if(abs(mavPos_[2]-targetPos_[2])<0.1)
    {
      node_state = MISSION_EXECUTION;
      std_srvs::SetBool set_traj;
      set_traj.request.data = true;
      set_traj.response.success = true;
      start_traj_client_.call(set_traj);
    }
    ros::spinOnce();
    break;
  }

  case LANDING: {
    geometry_msgs::PoseStamped landingmsg;
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose = home_pose_;
    landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
    target_pose_pub_.publish(landingmsg);
    node_state = LANDED;
    ros::spinOnce();
    break;
  }
  case LANDED:
    ROS_INFO("Landed. Please set to position control and disarm.");
    cmdloop_timer_.stop();
    break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}

void geometricCtrl::statusloopCallback(const ros::TimerEvent& event){
  if(sim_enable_){
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
        if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude){
  geometry_msgs::PoseStamped msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::imuCallback(const sensor_msgs::Imu& msg)
{
  ang_vel = toEigen(msg.angular_velocity);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd){
  mavros_msgs::AttitudeTarget msg;
  
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id= "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128; //Ignore orientation messages
  msg.thrust = cmd(3);
  
  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory(){
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;
  
  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory(){
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if(posehistory_vector_.size() > posehistory_window_){
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation){
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


void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd){

  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = targetAcc_;
  
  Eigen::Vector3d a_des;
  
  double yaw;
  if(velocity_yaw_) {
    mavYaw_ = std::atan2(-1.0 * mavVel_(1), mavVel_(0));
  }

  if(!gap_state){
  //  cout << targetPos_<<"\n";
  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - g_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);


  const Eigen::Vector3d pos_error = mavPos_ - targetPos_;
  const Eigen::Vector3d vel_error = mavVel_ - targetVel_;


  Eigen::Vector3d a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; //feedforward term for trajectory error
  if(a_fb.norm() > max_fb_acc_) a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; //Clip acceleration if reference is too large
  
  //const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * targetVel_; //Rotor drag
  
  //const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
  a_des = a_fb + a_ref - g_;

  q_des = acc2quaternion(a_des, mavYaw_);
  }

  else{
  q_des << cos(slit_ang), sin(slit_ang), 0, 0;
  a_des = - g_;
  }

  bodyrate_cmd = attcontroller(q_des, a_des, mavAtt_); //Calculate BodyRate

}

Eigen::Vector3d geometricCtrl::vee_map(const Eigen::Matrix3d R)
{
  Eigen::Vector3d v;

  v << -R(1,2),
        R(0,2),
       -R(0,1);

  return v;
}

Eigen::Vector4d geometricCtrl::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
   Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Matrix3d geometricCtrl::quat2RotMatrix(const Eigen::Vector4d q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d geometricCtrl::rot2Quaternion(const Eigen::Matrix3d R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d vector_acc, double yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / ( yb_des.cross(zb_des) ).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0),
            xb_des(1), yb_des(1), zb_des(1),
            xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

Eigen::Vector4d geometricCtrl::attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc, Eigen::Vector4d &curr_att){
  Eigen::Vector4d ratecmd;
  Eigen::Vector4d qe, q_inv, inverse;
  Eigen::Matrix3d rotmat;
  Eigen::Vector3d zb, eR;

///////////////////////////////////////////////////////////////////////////////////////
  if(controller == 2)
  {
    const Eigen::Matrix3d R     = quat2RotMatrix(curr_att);  
    const Eigen::Matrix3d R_des = quat2RotMatrix(ref_att);
    const Eigen::Vector3d ew    = ang_vel - targetW_;

    eR = 0.5*vee_map(R_des.transpose()*R - R.transpose()*R_des);

    Eigen::Vector3d d = Kr_.asDiagonal()*eR + Kw_.asDiagonal()*ew;

    ratecmd(0) = d(0);
    ratecmd(1) = d(1);
    ratecmd(2) = d(2);
  }
  else{
  inverse << 1.0, -1.0, -1.0, -1.0;
  q_inv = inverse.asDiagonal() * curr_att;
  qe = quatMultiplication(q_inv, ref_att);
  ratecmd(0) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(1);
  ratecmd(1) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(2);
  ratecmd(2) = (2.0 / attctrl_tau_) * std::copysign(1.0, qe(0)) * qe(3);
  }
  rotmat = quat2RotMatrix(mavAtt_);
  zb = rotmat.col(2);
  ratecmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_)); //Calculate thrust
///////////////////////////////////////////////////////////////////////////////////////

  return ratecmd;
}


void geometricCtrl::getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel){
  pos = mavPos_;
  att = mavAtt_;
  vel = mavVel_;
  angvel = mavRate_;

}

void geometricCtrl::getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel){
  pos = mavPos_ - targetPos_;
  vel = mavVel_ - targetVel_;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req,
                                          std_srvs::SetBool::Response &res){
  bool mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
}

void geometricCtrl::setBodyRateCommand(Eigen::Vector4d bodyrate_command){
  cmdBodyRate_= bodyrate_command;

}

void geometricCtrl::setFeedthrough(bool feed_through){
  feedthrough_enable_ = feed_through;

}


void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t  level) {

    if(max_fb_acc_ != config.max_acc){
		max_fb_acc_ = config.max_acc;
  		ROS_INFO("Reconfigure request : max_acc = %.2f ",config.max_acc);
    }
	else if(Kpos_x_ != config.Kp_x){
		Kpos_x_ = config.Kp_x;
	   ROS_INFO("Reconfigure request : Kp_x  = %.2f  ",config.Kp_x);
	}
	else if(Kpos_y_ != config.Kp_y){
		Kpos_y_ = config.Kp_y;
	   ROS_INFO("Reconfigure request : Kp_y  = %.2f  ",config.Kp_y);
	}
	else if(Kpos_z_ != config.Kp_z){
		Kpos_z_= config.Kp_z;
	   ROS_INFO("Reconfigure request : Kp_z  = %.2f  ",config.Kp_z);
	}
	else if(Kvel_x_ != config.Kv_x){
		Kvel_x_ = config.Kv_x;
	   ROS_INFO("Reconfigure request : Kv_x  = %.2f  ",config.Kv_x);
	}
	else if(Kvel_y_ != config.Kv_y){
		Kvel_y_ = config.Kv_y;
	   ROS_INFO("Reconfigure request : Kv_y =%.2f  ",config.Kv_y);
	}
	else if(Kvel_z_ != config.Kv_z){
		Kvel_z_ = config.Kv_z;
	   ROS_INFO("Reconfigure request : Kv_z  = %.2f  ",config.Kv_z);
	}

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_z_, -Kvel_z_;
}
