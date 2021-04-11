#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <body_msgs/PartMsg.h>
#include <body_lib/Body.h>
#include <support_msgs/HeartrateMsg.h>
#include <support_msgs/CalibrationMsg.h>
#include <support_lib/Utilities.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/InfoMsg.h>
#include <actionlib/server/simple_action_server.h>
#include <motioncontroll_action/MotionControllAction.h>
#include <motioncontroll_action/MotionControllGoal.h>
#include <motioncontroll_action/MotionControllResult.h>
#include <motioncontroll_action/MotionControllFeedback.h>

#define heartrateTopicName "Heartrate"

namespace Node{
    class HeartrateSubscriber : virtual protected ros::NodeHandle
    {
        protected:
            ros::Subscriber heartrateListner;
            ros::Duration heartrate;
        public:
            HeartrateSubscriber();
            HeartrateSubscriberCallback( support_msgs::HeartrateMsg::ConstPtr );
    };

    class MotionControllerNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            MotionController mc;
            ros::Publisher commandPublisher;          // to teensy
            ros::Publisher currentStatePublisher;     // to core / other nodes
            ros::Subscriber feedbackProcessorListner; // listening to FeedbackProcessor Node
            ros::Timer publisherTimer;
            actionlib::SimpleActionServer<motioncontroll_action::MotionControllAction> locomotionServer; // execute action from core
            std::string actionName;
            bool valid;

        public:
            MotionControllerNode( std::string publishCmdTopicName, std::string publishStateTopicName, std::string subscribeInfoTopicName, std::string locomotionActionName );
            void renewAllPublisherTimer () override;
            void processorListnerCallback ( body_msgs::PartMsg::ConstPtr );
            void locomotionActionCallback ( const motioncontroll_action::MotionControllAction::ConstPtr & );
            inline void publish_cmdMsg() const;
            inline void publish_feedback() const;
            inline void publish_currentState() const;
            bool valid();
    };

    class FeedbackProcessorNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            FeedbackProcessor fp;
            ros::Publisher ProcessedFeedbackPublisher;
            ros::Subscriber teencyListner;
			ros::Publisher currentStatePublisher;
            std::string nodeName;
            std::string publishTopicName;
            std::string subscribeTopicName;
			std::string heartrateFeedbackName;

        public:
            FeedbackProcessorNode( PartID ID, std::string NodeName, std::string PublishTopicName, std::string SubscribeTopicName, std::string HeartrateFeedbackName );
            void teencyListnerCallback( teensy_msgs::InfoMsg::ConstPtr );
            inline void publish_processedFeedback( body_msgs::PartMsg ) const ;
			void publish_CurrentState();
    };
}

#endif
