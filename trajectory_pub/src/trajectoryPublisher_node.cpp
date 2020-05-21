
#include "trajectory_publisher/trajectoryPublisher.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_pub");
    ros::NodeHandle nh("");

    trajectoryPublisher referencePublisher(nh);
    ros::spin();
    return 0;
}
