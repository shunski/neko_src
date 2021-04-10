#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <body/Body.h>
#include <support_msgs/HeartrateMsg.h>
#include <actionlib/client/simple_action_server.h>
#include <motioncontroll_action/MotionControllAction.h>
#include <motioncontroll_action/MotionControllGoal.h>
#include <motioncontroll_action/MotionControllResult.h>
#include <motioncontroll_action/MotionControllFeedback.h>
#include <support_msgs/CalibrationMsg.h>
#include <body_msgs/PartMsg.h>

#define heartrateTopicName "Heartrate"

namespace Node{
    class HeartrateSubscriber : virtual protected ros::NodeHandle
    {
        protected:
            ros::Subscriber heartrateListner;
            ros::Duration heartrate;
        public:
            HeartrateSubscriber();
            HeartrateSubscriberCB( support_msgs::HeartrateMsg::ConstPtr );
    };

    class MotionControllerNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            MotionController mc;
            ros::Publisher commandPublisher;
            ros::Publisher currentStatePublisher;
            ros::Subscriber infoProcessorListner;
            ros::Timer publisherTimer;
            actionlib::SimpleActionServer<motioncontroll_action::MotionControllAction> locomotionServer;
            std::string actionName;

        public:
            MotionControllerNode(std::string publishCmdTopicName, std::string publishStateTopicName, std::string subscribeInfoTopicName, std::string locomotionActionName);
            void renewAllPublisherTimer () override;
            void processorListnerCallback ( body_msgs::PartMsg::ConstPtr );
            void locomotionActionCallback ( const motioncontroll_action::MotionControllAction::ConstPtr & );
            inline void publish_cmdMsg() const;
            inline void publish_feedback() const;
            void publish_currentState() const;
    };

    class FeedbackProcessorNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            FeedbackProcessor fp;
            ros::Publisher legProcessedFeedbackPublisher;
            ros::Subscriber teencyListner;
			ros::Publisher currentStatePublisher;
            std::string nodeName;
            std::string publishTopicName;
            std::string subscribeTopicName;
			std::string heartrateFeedbackName;

        public:
            FeedbackProcessorNode( PartID ID, std::string NodeName, std::string PublishTopicName, std::string SubscribeTopicName, std::string HeartrateFeedbackName );
            void teencyListnerCallback ( teensy_msgs::InfoMsg::ConstPtr );
            inline void publish_processedFeedback( body_msgs::PartMsg ) const ;
			void publish_CurrentState();
    };
}

#endif
