

#include "trajectory_publisher/eth_trajectory.h"

mav_trajectory_generation::Trajectory trajectory;

Eigen::Vector3d init_pos, mid_pos, final_pos;

Eigen::Vector3d init_vel, mid_vel, final_vel;

double T_ = 0;

void eth_set_pos(Eigen::Vector3d p_init, Eigen::Vector3d p_final)
{
	init_pos  = p_init;
	final_pos = p_final;
}

void eth_set_vel(Eigen::Vector3d v_init, Eigen::Vector3d v_final)
{
	init_vel  = v_init;
	final_vel = v_final;  
}

void get_mid_pos_vel()
{
	mid_pos = (init_pos + final_pos)/2;
	mid_vel << 0,0,0;
}

double eth_trajectory_init()
{
mav_trajectory_generation::Vertex::Vector vertices;
const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension), flip_node1(dimension), flip_node2(dimension), flip_node3(dimension);

get_mid_pos_vel();

start.makeStartOrEnd(init_pos, derivative_to_optimize);
start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, init_vel);
vertices.push_back(start);

Eigen::Matrix3d R;


  R << 1, 0                   , 0                ,
       0, cos(90*M_PI/180.0), -sin(90*M_PI/180.0),
       0, sin(90*M_PI/180.0), cos(90*M_PI/180.0) ;

  Eigen::Vector3d g_(0.0, 0.0, 9.81);

  Eigen::Vector3d final_acc = R*g_ - g_;

  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, final_pos);
  middle.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, final_acc);
  vertices.push_back(middle);

  Eigen::Vector3d vel_end(0.0,0.0,0.0);
  Eigen::Vector3d pos_end(8.0,0.0,10.0);

  end.makeStartOrEnd(pos_end, derivative_to_optimize); 
  end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, vel_end);
  vertices.push_back(end);


////////////////////////////////////////////////////////////////////////////////////////////
//FOR FLIP
/*
  //middle.makeStartOrEnd(final_pos, derivative_to_optimize); 
  middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, final_pos);
  //vertices.push_back(middle);

  Eigen::Matrix3d R;
  Eigen::Vector3d final_acc;
  Eigen::Vector3d g_(0.0, 0.0, 9.81);

  R << 1, 0                 ,  0                  ,
       0, cos(180*M_PI/180.0), -sin(180*M_PI/180.0),
       0, sin(180*M_PI/180.0), cos(180*M_PI/180.0) ;

  final_pos << 0, 0, 13;
  final_acc = R*g_ - g_;

  //flip_node.addConstraint(mav_trajectory_generation::derivative_order::POSITION, final_pos);
  flip_node1.addConstraint(mav_trajectory_generation::derivative_order::POSITION, final_pos); 
  flip_node1.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, final_acc);
  vertices.push_back(flip_node1);


  final_pos << 0, 0.0, 10.0;
  final_acc << 0, 0, 0;
  //flip_node.addConstraint(mav_trajectory_generation::derivative_order::POSITION, final_pos);
  end.makeStartOrEnd(final_pos, derivative_to_optimize);
  end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, final_acc); 
  vertices.push_back(end);
*/
/////////////////////////////////////////////////////////////////////////////////////////
  
std::vector<double> segment_times;
const double v_max = 20.0;
const double a_max = 10.0;
segment_times = estimateSegmentTimes(vertices, v_max, a_max);

for(int i=0;i<segment_times.size();i++)
	T_ += segment_times.at(i);

const int N = 10;
//const int N = 6;
mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
opt.solveLinear();

mav_trajectory_generation::Segment::Vector segments;
opt.getSegments(&segments);

opt.getTrajectory(&trajectory);
return T_;
}

Eigen::Vector3d eth_trajectory_pos(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_vel(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::VELOCITY;
return trajectory.evaluate(time, derivative_order);
}


Eigen::Vector3d eth_trajectory_acc(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::ACCELERATION;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d eth_trajectory_jerk(double time)
{
int derivative_order = mav_trajectory_generation::derivative_order::JERK;
return trajectory.evaluate(time, derivative_order);
}

Eigen::Vector3d eth_trajectory_angvel(double time)
{
  Eigen::Vector3d acc, jerk, h, zb, w, xc, yb, xb;

  float m = 0.95;

  Eigen::Vector3d g;

  g << 0, 0, 9.8;
  
  acc  = eth_trajectory_acc(time) + g;
  jerk = eth_trajectory_jerk(time);
  
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

nav_msgs::Path getSegment(){

  Eigen::Vector3d targetPosition;
  Eigen::Vector4d targetOrientation;
  nav_msgs::Path segment;

  targetOrientation << 1.0, 0.0, 0.0, 0.0;
  geometry_msgs::PoseStamped targetPoseStamped;

  for(double t = 0 ; t < T_ ; t+=0.01){
    targetPosition = eth_trajectory_pos(t);
    targetPoseStamped = vector3d2PoseStampedMsg(targetPosition, targetOrientation);
    segment.poses.push_back(targetPoseStamped);
  }
  return segment;
}

geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position, Eigen::Vector4d orientation){
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