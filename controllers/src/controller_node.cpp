//The Node can run with different controllers. The controller which needs to be
//taken is specified in the launch file

#include "controllers/controller.h"
#include "controllers/eth_quat_controller.h"


int main(int argc, char** argv) {
  ros::init(argc,argv,"controller_node");
  ros::NodeHandle nh("");

  controller *control_obj = new controller(nh);

  ros::spin();
  return 0;
}
