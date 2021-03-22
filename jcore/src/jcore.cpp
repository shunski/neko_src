// jcore.cpp

#include "ros/ros.h"
#include <jcatty_msgs/heartrate.h>
#include <iostream>

int main(int argc, char **argv){
    ros::init(argc, argv, "jcore");
    ros::NodeHandle nh;
    ros::Publisherpub = nh.advertise<jcatty_msgs::heartrate>("/jcore_heartrate", 10);

    ros::Rate loop_rate = 10;

    while (ros::ok()){
　　　　ros::Subscriber rh_info = nh.subscribe("/motion_info", 10, motion_info_callback());
	
        Part rf(), lf(), rh(), rf();
        jcatty_msgs::heartrate msg;
	msg.rate = 1;
	msg.
    }
}
