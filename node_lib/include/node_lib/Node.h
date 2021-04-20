#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <body_msgs/PartMsg.h>
#include <body_lib/Body.h>
#include <std_msgs/Bool.h>
#include <support_msgs/HeartrateMsg.h>
#include <support_msgs/CalibrationMsg.h>
#include <support_lib/Utilities.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>
#include <actionlib/server/simple_action_server.h>
#include <motioncontroll_action/MotionControllAction.h>
#include <motioncontroll_action/MotionControllGoal.h>
#include <motioncontroll_action/MotionControllResult.h>
#include <motioncontroll_action/MotionControllFeedback.h>
#include <support_msgs/ActionStartNotifierMsg.h>
#include <support_msgs/ActionEndReporterMsg.h>

#define heartrateTopicName "Heartrate"

namespace Node{
    class HeartrateSubscriberNode : virtual protected ros::NodeHandle
    {
        protected:
            ros::Subscriber heartrateListner;
            ros::Duration heartrate;
        public:
            HeartrateSubscriberNode();
            void HeartrateSubscriberCallback( const support_msgs::HeartrateMsg::ConstPtr & );
			virtual void renewAllPublisherTimer()=0;
    };



    class MotionControllerNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriberNode
    {
        protected:
            Body::MotionController mc;
            ros::Publisher commandPublisher;             // to teensy
            ros::Publisher currentStatePublisher;        // to core / other nodes
            ros::Publisher actionStartNotifier;
			ros::Subscriber actionEndListner;
            ros::Subscriber feedbackProcessorListner;    // listening to FeedbackProcessor Node
            ros::Timer currentStatePublisherTimer;
            actionlib::SimpleActionServer<motioncontroll_action::MotionControllAction> locomotionServer; // execute action from core

            std::string nodeName;
            std::string locomotionActionName;
            std::string fromFeedbackProcessorFeedbackTopicName;
			std::string fromFeedbackProcessorFinishActionTopicName;
            std::string toTeensyPublishTopicName;
            std::string toFeedbackProcessorActionStartNotifierName;
            std::string heartrateFeedbackName;
            bool valid;
            ros::Timer mcValidnessSensor;

        public:
            MotionControllerNode( PartID pID, std::string PartName );
            void renewAllPublisherTimer () override;
            void processorListnerCallback ( const body_msgs::PartMsg::ConstPtr & );
            void locomotionActionCallback ( const motioncontroll_action::MotionControllGoalConstPtr & );
            void publish_CommandMsg() const;
            void publish_feedback();
            void publish_currentState() const ;
			void end_action( const support_msgs::ActionEndReporterMsg::ConstPtr & );
            void checkMcValidness();
			bool isValid() const ;
    };



    class FeedbackProcessorNode : virtual protected ros::NodeHandle, virtual protected HeartrateSubscriberNode
    {
        protected:
            Body::FeedbackProcessor fp;
            ros::Publisher processedFeedbackPublisher;
            ros::Subscriber teensyListner;
            ros::Subscriber teensyCommandSubscriber;
			ros::Publisher currentStatePublisher;
            ros::Timer currentStatePublisherTimer;
            ros::Subscriber actionStartListner;
			ros::Publisher actionEndReporter;

            std::string nodeName;
            std::string toMotionControllerFeedbackTopicName;
            std::string toMotionControllerFinishActionTopicName;
            std::string fromMotionControllerTeensyCommandTopicName;
            std::string fromTeensySubscribeTopicName;
            std::string fromMotionControllerActionStartNotifierTopicName;
			std::string heartrateFeedbackName;

            bool valid;
            ros::Timer fpValidnessSensor;

        public:
            FeedbackProcessorNode( PartID pID, std::string PartName );
			void teensyCommandListnerCallback( const teensy_msgs::CommandMsg::ConstPtr & );
            void teensyListnerCallback( const teensy_msgs::FeedbackMsg::ConstPtr & );
            inline void publish_processedFeedback( body_msgs::PartMsg ) const ;
			void publish_currentState();
            void checkFpValidness();
            void actionStartListnerCallback( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg );
			bool isValid() const ;
			void renewAllPublisherTimer() override ;
    };
}

#endif
