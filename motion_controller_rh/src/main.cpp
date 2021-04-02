// main.cpp
#include <ros/ros.h>
#include "McNodeRH.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "rh_motion_controller");
    McNodeRH rhNode();
    ros::spin();
    return 0;
}
