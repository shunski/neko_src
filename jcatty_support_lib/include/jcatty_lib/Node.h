#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Body.h>
#include <jcatty_support_msgs/HeartrateMsg.h>
#include <actionlib/client/simple_action_server.h>
#include <jcatty_action/MotionControllAction.h>
#include <jcatty_action/MotionControllGoal.h>
#include <jcatty_action/MotionControllResult.h>
#include <jcatty_action/MotionControllFeedback.h>
#include <jcatty_msgs/CalibrationMsg.h>
#include <jcatty_partMsgs/PartMsg.h>

#define heartrateTopicName "Heartrate"

namespace Node{
    class HeartrateSubscriber : virtual protected ros::NodeHandle
    {
        protected:
            ros::Subscriber heartrateListner;
            ros::Duration heartrate;
        public:
            HeartrateSubscriber();
            HeartrateSubscriberCB(jcatty_support_msgs::HeartrateMsg::ConstPtr );
    };

    class MotionControllerNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            MotionController mc;
            ros::Publisher legCmdPublisher;
            ros::Publisher currentStatePublisher;
            ros::Subscriber legInfoProcessorListner;
            ros::Timer publisherTimer;
            actionlib::SimpleActionServer<jcatty_locomotion_action::MotionControllAction> locomotionServer;
            std::string actionName;

        public:
            MotionControllerNode(std::string publishCmdTopicName, std::string publishStateTopicName, std::string subscribeInfoTopicName, std::string locomotionActionName);
            void renewAllPublisherTimer () override;
            void processorListnerCallback ( jcatty_body_msgs::PartMsg::ConstPtr );
            void locomotionActionCallback ( const jcatty_locomotion_action::MotionControllAction::ConstPtr & );
            inline void publish_cmdMsg() const;
            inline void publish_feedback() const;
            void publish_currentState() const;
    };

    class FeedbackProcessorNode : virtual protected ros::NodeHandle
    {
        protected:
            FeedbackProcessor fp;
            ros::Publisher legProcessedFeedbackPublisher;
            ros::Subscriber teencyListner;
            std::string nodeName;
            std::string publishTopicName;
            std::string subscribeTopicName;

        public:
            FeedbackProcessorNode(std::string NodeName, std::string PublishTopicName, std::string SubscribeTopicName);
            void teencyListnerCallback ( jcatty_teensy_msgs::InfoMsg::ConstPtr );
            inline void publish_processedFeedback( jcatty_body_msgs::PartMsg ) const;
    };
}

#endif
