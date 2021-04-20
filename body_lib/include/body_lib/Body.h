// Body.h
#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <queue>
#include <tuple>
#include <limits.h>
#include <actionlib/server/simple_action_server.h>
#include <motioncontroll_action/MotionControllAction.h>
#include <motioncontroll_action/MotionControllGoal.h>
#include <motioncontroll_action/MotionControllResult.h>
#include <motioncontroll_action/MotionControllFeedback.h>
#include <support_msgs/CalibrationMsg.h>
#include <support_msgs/ActionStartNotifierMsg.h>
#include <support_msgs/ActionEndReporterMsg.h>
#include <parts_lib/CattyParts.h>
#include <body_msgs/PartCommandMsg.h>
#include <body_msgs/PartMsg.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>

namespace Body{


    class Part
    {
        private:
            const PartID part_id;
            const Uint16 scene_id;

            std::vector<KondoServo> kondoServoSet;
            std::vector<BrushedMotor> brushedMotorSet;
            std::vector<BrushlessMotor> brushlessMotorSet;
            std::vector<MotionSensor> motionSensorSet;

			bool valid;
			bool well_defined;
			ObjectState state;

        public:
            Part();
		    Part( PartID );
            Part( PartID, Uint16 SceneID );
            Part( const teensy_msgs::CommandMsg::ConstPtr & );
            Part( const teensy_msgs::FeedbackMsg::ConstPtr & );
			Part( const body_msgs::PartMsg::ConstPtr & );
            Part( PartID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum );
            Part( PartID, const std::vector<KondoServo>&,
                          const std::vector<BrushedMotor>&,
                          const std::vector<BrushlessMotor>& );
			Part( PartID, const std::vector<parts_msgs::KondoServoCommandMsg> &,
                          const std::vector<parts_msgs::BrushedMotorCommandMsg> &,
                          const std::vector<parts_msgs::BrushlessMotorCommandMsg> & );
            CattyError set( const teensy_msgs::FeedbackMsg::ConstPtr & );
            CattyError set( const teensy_msgs::CommandMsg::ConstPtr & );
			CattyError set( const body_msgs::PartMsg::ConstPtr & );

            CattyError set_PartMsg( body_msgs::PartMsg & );
			CattyError set_CommandMsg( teensy_msgs::CommandMsg & );

			bool isValid() const ;
			bool isWellDefined() const ;
            Uint16 get_scene_id() const ;
    };



    class Body
    {
        private:
            Part right_fore;
            Part left_fore;
            Part right_hind;
            Part left_hind;
            Part waist;
            Part chest;
            Part head;

        public:
            Body();
    };



    class MotionController
    {
        protected:
			const PartID part_id;
			Uint16 sequenceSize;
			Uint16 currentSceneIdProcessed;                     // keeping track of how many scenes are processed
            std::vector<Part> expectedStates;
            Part actualCurrentState;
            std::vector<Part>::iterator expectedCurrentState;
            std::queue<body_msgs::PartMsg> processedFeedbackPending;

			motioncontroll_action::MotionControllFeedback motionControllFeedback;
			motioncontroll_action::MotionControllResult motionControllResult;

            const ros::Duration expectedSceneDuration;
            std::queue<ros::Duration> actualCurrentSceneDuration;
            ros::Time timeOfActionStart;
            ros::Time timeOfLastAction;

            bool inAction;
			bool valid;

        public:
            MotionController ( PartID );
            CattyError set_action( const motioncontroll_action::MotionControllGoal::ConstPtr & );
            void proceed();
			// void startInitializationAction( initialize_service::PartInitialization::Request &,
			//								initialize_servoce::PartInitialization::Response & );
			void startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & );
            CattyError set_CommandMsg( teensy_msgs::CommandMsg & ) const ;
            bool isEnd() const ;
            CattyError initialize_feedbackMsg();
            CattyError initialize_resultMsg();
            CattyError update_locomotionActionFeedbackMsg( const body_msgs::PartMsg::ConstPtr &, const std::string & nodeName );
            CattyError update_locomotionActionResultMsg( const support_msgs::ActionEndReporterMsg::ConstPtr &, const std::string & );
			motioncontroll_action::MotionControllFeedback get_locomotionActionFeedbackMsg() const;
			motioncontroll_action::MotionControllResult get_locomotionActionResultMsg() const;
            ros::Duration get_actualCurrentSceneDuration() const;
			ros::Duration get_expectedSceneDuration() const;
            CattyError set_actionStartNotifier( support_msgs::ActionStartNotifierMsg & );
            void add_processedFeedbackPending( const body_msgs::PartMsg::ConstPtr & );
            void end_action();
            bool isInAction() const ;
			bool isValid() const;

    };



    class FeedbackProcessor
    {
        protected:
			const PartID part_id;
            std::queue<Part> pendingScenes;
            Uint16 sequenceSize;
            Uint16 currentSceneIdReceived;
            Uint16 currentSceneIdProcessed;
            Uint16 numMissingExpectedScenes;              // # of scenes heard from teensy but not from MotionController
            Uint16 numMissingActualScenes;                // # of scenes heard from MotionController but not from teensy
            Uint16 numTotallyMissingScenes;               // # of scenes not processed at all which were supposed to be there (guessed by scene_id of messages)
            std::vector<ObjectState> stateOfScenes;
            bool inAction;
            bool valid;

        public:
            FeedbackProcessor( PartID );
            CattyError start_action( const support_msgs::ActionStartNotifierMsg::ConstPtr &, const std::string & NodeName );
            bool add_pendingScenes( const teensy_msgs::CommandMsg::ConstPtr & ); // When command is received from MotionController. return true if the action ends.
            Part process_Feedback( const teensy_msgs::FeedbackMsg::ConstPtr & ); // When feedback is received from teensy
            void reset();
            CattyError set_ActionEndReporterMsg( support_msgs::ActionEndReporterMsg & );
            void fillTheRest();
            bool isInAction() const ;
            bool isValid() const ;
    };
}

#endif
