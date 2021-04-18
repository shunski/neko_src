#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <body_msgs/PartMsg.h>
#include <body_lib/Body.h>
#include <support_msgs/HeartrateMsg.h>
#include <support_msgs/CalibrationMsg.h>
#include <support_srv/LocomotionActionStateSrv.h>
#include <support_lib/Utilities.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>
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
            void HeartrateSubscriberCallback( support_msgs::HeartrateMsg::ConstPtr );
			virtual void renewAllPublisherTimer();
    };

    class MotionControllerNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            Body::MotionController mc;
            ros::Publisher commandPublisher;          // to teensy
            ros::Publisher currentStatePublisher;     // to core / other nodes
            ros::Subscriber feedbackProcessorListner; // listening to FeedbackProcessor Node
            ros::Timer currentStatePublisherTimer;
            actionlib::SimpleActionServer<motioncontroll_action::MotionControllAction> locomotionServer; // execute action from core
            std::string nodeName;
            std::string locomotionActionName;
            std::string fromFeedbackProcessorSubscribeTopicName;
            std::string toTeensyPublishTopicName;
            std::string toFeedbackProcessorTopicName;
            std::string heartrateFeedbackName;
            bool valid;
            ros::Timer mcValidnessSensor;

        public:
            MotionControllerNode( PartID pID, std::string PartName );
            void renewAllPublisherTimer () override;
            void processorListnerCallback ( const body_msgs::PartMsg::ConstPtr );
            void locomotionActionCallback ( const motioncontroll_action::MotionControllAction::ConstPtr & );
            inline void publish_cmdMsg() const;
            inline void publish_feedback() const;
            inline void publish_currentState() const;
            void checkMcValidness();
    };

    class FeedbackProcessorNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriber
    {
        protected:
            Body::FeedbackProcessor fp;
            ros::Publisher ProcessedFeedbackPublisher;
            ros::Subscriber teensyListner;
			ros::Publisher currentStatePublisher;
            ros::Timer currentStatePublisherTimer;
            ros::ServiceServer actionStateServer;
            std::string nodeName;
            std::string toMotionControllerPublishTopicName;
            std::string fromTeensySubscribeTopicName;
            std::string fromMotionControllerServiceTopicName;
			std::string heartrateFeedbackName;
            bool valid;
            ros::Timer fpValidnessSensor;

        public:
            FeedbackProcessorNode( PartID pID, std::string PartName );
            void teensyListnerCallback( teensy_msgs::InfoMsg::ConstPtr );
            inline void publish_processedFeedback( body_msgs::PartMsg ) const ;
			void publish_CurrentState();
            void checkFpValidness();
    };
}

#endif
