// main.cpp

#include <ros/ros.h>
#include <jcatty_msgs/rh_info.h>
#include <iostream>
#include "McNode.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_controller");
    RH_McNode McNode();
    ros::Timer cmdTimer = McNode.get_nodeHandle().createTimer(McNode.get_heartrateAsDuration(),
                                                            & RH_McNode::publishCmd,
                                                            RH_McNode);
    ros::Timer calibrationTimer = McNode.get_nodeHandle().createTimer(RH_McNode.get_heartrateAsDuration(),
                                                            & RH_McNode::publishCurrentState,
                                                            RH_McNode);

    ros::spin();
    return 0;
}
