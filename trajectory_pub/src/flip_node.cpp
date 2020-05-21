
#include "trajectory_publisher/flip_trajectory.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "flip_traj_node");
    ros::NodeHandle nh("");

    fliptraj flip_traj_node(nh);
    ros::spin();
    return 0;
}
