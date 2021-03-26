// McNode.h implemented by McNode.cpp

#ifndef MCNODE_H
#define MCNODE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_server.h>
#include <jcatty_action/MotionControllAction.h>
#include <jcatty_action/MotionControllGoal.h>
#include <jcatty_action/MotionControllResult.h>
#include <jcatty_action/MotionControllFeedback.h>
#include <jcatty_msgs/CalibrationMsg.h>

#define queue_size = 100;

template<typename CmdMsg, typename InfoMsg>
class McNode
{
    protected:
        ros::NodeHandle nh;
        MotionControllerRH mc;

        actionlib::SimpleActionServer<jcatty_action::MotionControll> core_server;
        jcatty_action::MotionControll result;
        jcatty_action::MotionControll feedback;

        ros::Publisher calibrationPublisher;
        ros::Publisher legCmdPublisher;
        ros::Subscriber LegInfoSubscriber;

        jcatty_lib::Part currentState;
        CmdMsg commandForLegs;

        CattyLocomotion::


    public:
        McNode() : core_server( nh, "RH_MotionControll", boost::bind( &MotionControllAction::core_callback, this, _1 ), false )){
            MotionControll.start();
            calibrationPublisher = nh.advertise<jcatty_msgs::calibrationMsg>()
            legCmdPublisher = nh.advertise<CmdMsg>( "rhCommand", queue_size );
            LegInfoSubscriber = nh.subscribe<InfoMsg>( "rhInfo", queue_size, &McNode::rh_callback(), this );
        }

        virtual void legInfoCallback( const typename CmdMsg::ConstPtr& msg ) = 0;
        void publishLegCmd(){
            CattyLocomotion.set_cmd();
        }

        void core_callback( const jcatty_action::RH_MotionControllGoalConstPtr & goal );
};


#endif
