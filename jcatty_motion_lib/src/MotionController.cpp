// MotionController.cpp implements MotionController.h

#include <ros/ros.h>
#include <jcatty_msgs/rh_info.h>


McNode::McNode(){
    rf = new rightFore();
    lf = new leftFore()
    rh = new rightHind();
    lh = new leftHind();
    chest = new Chest();
    waist = new Waist();
}

