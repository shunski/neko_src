// McNode.h implemented by McNode.cpp

#ifndef MCNODERH_H
#define MCNODERH_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_server.h>
#include <jcatty_msgs/rhCmd.h>
#include <jcatty_msgs/rhInfo.h>

#define queue_size = 100;

class McNode
{
    private:
        ros::NodeHandle nh;
        MotionController mc;

        actionlib::SimpleActionServer<jcatty_action::MotionControllRH> core_server;
        jcatty_action::MotionControll result;
        jcatty_action::MotionControll feedback;

        ros::Publisher rh_publisher;
        ros::Subscriber rh_subscriber;

        jcatty_lib::Part currentState;


    public:
        McNode() : core_server( nh, "MotionControllRH", boost::bind( &MotionControllAction::core_callback, this, _1 ), false )){
            MotionControll.start();
            rh_publisher = nh.advertise<jcatty_msgs::rhCmd>( "CommandRH", queue_size );
            rh_subscriber = nh.subscribe<jcatty_msgs::rhInfo>( "infoRH", queue_size, &McNode::rh_callback(), this );
        }

        void rf_callback( const jcatty_action::rfInfo::ConstPtr& msg );
        void core_callback( const jcatty_action::MotionControllRHGoalConstPtr &goal );
};


#endif
