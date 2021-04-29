#ifndef NODE_H
#define NODE_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include "body_msgs/PartMsg.h"
#include "body_msgs/ProcessedFeedbackMsg.h"
#include "body_lib/Body.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "support_msgs/HeartrateMsg.h"
#include "support_msgs/CalibrationMsg.h"
#include "support_srvs/RegistrationSrv.h"
#include "support_lib/Utilities.h"
#include "teensy_msgs/CommandMsg.h"
#include "teensy_msgs/FeedbackMsg.h"
#include "actionlib/server/simple_action_server.h"
#include "motioncontroll_action/MotionControllAction.h"
#include "motioncontroll_action/MotionControllGoal.h"
#include "motioncontroll_action/MotionControllResult.h"
#include "motioncontroll_action/MotionControllFeedback.h"
#include "support_msgs/ActionStartNotifierMsg.h"
#include "support_msgs/ActionEndReporterMsg.h"

const std::string heartrateTopicName = "Heartrate";
const std::string nodeRegistrationTopicName = "NodeRegistration";

namespace node{
    class GenericCattyNode : virtual protected ros::NodeHandle
    {
        private:
            bool initSucceeded;
            ros::Publisher heartSoundPublisher;
        protected:
            std::string nodeName;
            std::string currentStateTopicName;

            GenericCattyNode( const std::string & NodeName );
            void publish_heartSound() const;
        public:
            bool didInitSucceed() const;
    };


    class HeartratePublisherNode : virtual protected ros::NodeHandle
    {
        protected:
            ros::Publisher heartratePublisher;
            ros::Duration heartrate;

        public:
            HeartratePublisherNode();
            void publishHeartrate();
            CattyError set_heartrate( const ros::Duration & );
            CattyError set_heartrate( const double );
            void publish_newHeartrate();
    };


    class HeartrateSubscriberNode : virtual protected ros::NodeHandle
    {
        protected:
            ros::Subscriber heartrateListner;
            ros::Duration heartrate;
        public:
            HeartrateSubscriberNode();
            void heartrateSubscriberCallback( const support_msgs::HeartrateMsg::ConstPtr & );
			virtual void heartPumped()=0;
    };


    class RegistrarNode : virtual protected ros::NodeHandle, public HeartrateSubscriberNode
    {
        protected:
            ros::ServiceServer registrationServer;
            std::map<std::string, std::tuple<ros::Subscriber, ros::Time> > registeredNodeList;
        public:
            RegistrarNode();
            void disregister_disqualifiedNode();
            std::vector<std::string> get_allLiveNodes();
            bool isRegistered( const std::string & );
            bool registrationCallback( support_srvs::RegistrationSrv::Request &, support_srvs::RegistrationSrv::Response & );
            void heartrateSoundSubscriberCallback( const std_msgs::String::ConstPtr & msg );
            void heartPumped() override;
    };



    class MotionControllerNode : virtual protected ros::NodeHandle, public GenericCattyNode, public HeartrateSubscriberNode
    {
        protected:
            body::MotionController mc;
            ros::Publisher commandPublisher;             // to teensy
            ros::Publisher currentStatePublisher;        // to core / other nodes
            ros::Publisher actionStartNotifier;
            ros::Subscriber actionEndListner;
            ros::Subscriber feedbackProcessorListner;    // listening to FeedbackProcessor Node

            std::string locomotionActionName;
            std::string fromFeedbackProcessorFeedbackTopicName;
            std::string fromFeedbackProcessorFinishActionTopicName;
            std::string toTeensyPublishTopicName;
            std::string toFeedbackProcessorActionStartNotifierName;
            std::string heartPumpedTopicName;

            actionlib::SimpleActionServer<motioncontroll_action::MotionControllAction> locomotionServer; // execute action from core

            bool valid;
            ros::Timer mcValidnessSensor;

        public:
            MotionControllerNode( PartID pID, std::string PartName );
            void heartPumped () override;
            void processorListenerCallback ( const body_msgs::ProcessedFeedbackMsg::ConstPtr & );
            void locomotionActionCallback ( const motioncontroll_action::MotionControllGoalConstPtr & );
            void publish_CommandMsg() const;
            void publish_feedback();
            void publish_currentState() const ;
			void end_action( const support_msgs::ActionEndReporterMsg::ConstPtr & );
            void checkMcValidness();
			bool isValid() const ;
    };




    class FeedbackProcessorNode : virtual protected ros::NodeHandle, public GenericCattyNode, public HeartrateSubscriberNode
    {
        protected:
            body::FeedbackProcessor fp;
            ros::Publisher processedFeedbackPublisher;
            ros::Subscriber teensyListner;
            ros::Subscriber teensyCommandSubscriber;
			ros::Publisher currentStatePublisher;
            ros::Timer currentStatePublisherTimer;
            ros::Subscriber actionStartListner;
			ros::Publisher actionEndReporter;
            ros::Timer actionEndTimer;

            std::string toMotionControllerFeedbackTopicName;
            std::string toMotionControllerFinishActionTopicName;
            std::string fromMotionControllerTeensyCommandTopicName;
            std::string fromTeensySubscribeTopicName;
            std::string fromMotionControllerActionStartNotifierTopicName;
			std::string heartPumpedTopicName;

            bool valid;
            ros::Timer fpValidnessSensor;

        public:
            FeedbackProcessorNode( PartID pID, std::string PartName );
			void teensyCommandListenerCallback( const teensy_msgs::CommandMsg::ConstPtr & );
            void end_action();
            void teensyListenerCallback( const teensy_msgs::FeedbackMsg::ConstPtr & );
            inline void publish_processedFeedback( body_msgs::PartMsg ) const ;
			void publish_currentState();
            void checkFpValidness();
            void actionStartListenerCallback( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg );
			bool isValid() const ;
			void heartPumped() override ;
    };
}

#endif
