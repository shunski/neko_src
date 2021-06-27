// Body.h
#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <queue>
#include <tuple>
#include <limits.h>
#include "actionlib/server/simple_action_server.h"
#include "support_lib/support_lib.h"
#include "parts_lib/actuator.h"
#include "body_lib/MotionControllAction.h"
#include "body_lib/MotionControllGoal.h"
#include "body_lib/MotionControllResult.h"
#include "body_lib/MotionControllFeedback.h"
#include "body_lib/BodyCommandMsg.h"
#include "body_lib/BodyMsg.h"
#include "body_lib/ProcessedFeedbackMsg.h"
#include "teensy_msgs/CommandMsg.h"
#include "teensy_msgs/FeedbackMsg.h"

namespace body{


    class Body
    {
        private:
            const PartID part_id;
            Uint16 scene_id;

            AbstructBody abstract_body;
            std::vector<Component> concrete_body;

			bool valid;
			bool well_defined;
			ObjectState state;

        public:
            Body();
		    Body( PartID );
            Body( PartID, Uint16 SceneID );
            Body( const teensy_msgs::CommandMsg::ConstPtr & );
            Body( const teensy_msgs::FeedbackMsg::ConstPtr & );
			Body( const body_msgs::PartMsg::ConstPtr & );
            Body( PartID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum );
            Body( PartID, const std::vector<KondoServo>&,
                          const std::vector<BrushedMotor>&,
                          const std::vector<BrushlessMotor>& );
			Body( Uint16 sceneId, PartID,
                    const std::vector<parts_msgs::KondoServoCommandMsg> &,
                    const std::vector<parts_msgs::BrushedMotorCommandMsg> &,
                    const std::vector<parts_msgs::BrushlessMotorCommandMsg> & );
            Body( const Part & );

            void operator=( const Part & );

            PartID get_part_id() const ;
            Uint16 get_scene_id() const ;
            ObjectState get_state() const ;

            CattyError set( const teensy_msgs::FeedbackMsg::ConstPtr & );
            CattyError set( const teensy_msgs::CommandMsg::ConstPtr & );
			CattyError set( const body_msgs::PartMsg::ConstPtr & );

            CattyError set_PartMsg( body_msgs::PartMsg & );
            CattyError set_PartCommandMsg( body_msgs::PartCommandMsg & );
			CattyError set_CommandMsg( teensy_msgs::CommandMsg & );

			bool isValid() const ;
			bool isWellDefined() const ;

            void set_RandomPartCommandMsg( body_msgs::PartCommandMsg & );
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

			std::queue<motioncontroll_action::MotionControllFeedback> motionControllFeedbacks;
			motioncontroll_action::MotionControllResult motionControllResult;

            ros::Duration expectedSceneDuration;
            ros::Time timeOfActionStart;

            bool inAction;
			bool valid;

        public:
            MotionController ( PartID );
            CattyError set_action( const motioncontroll_action::MotionControllGoal::ConstPtr & );
            void proceed();
			void startMotioncontrollAction();
            CattyError set_CommandMsg( teensy_msgs::CommandMsg & ) const ;
            bool isEnd() const ;
            void initialize_feedbackMsg();
            void initialize_resultMsg();
            CattyError update_locomotionActionFeedbackMsg( const body_msgs::ProcessedFeedbackMsg::ConstPtr &, const std::string & nodeName );
            CattyError update_locomotionActionResultMsg( const support_msgs::ActionEndReporterMsg::ConstPtr &, const std::string & );
			bool set_locomotionActionFeedbackMsg( motioncontroll_action::MotionControllFeedback & );
			motioncontroll_action::MotionControllResult get_locomotionActionResultMsg() const;
			ros::Duration get_expectedSceneDuration() const;
            CattyError set_actionStartNotifier( support_msgs::ActionStartNotifierMsg & );
            void end_action();
            void reset();
            bool isInAction() const ;
			bool isValid() const;

    };



    class FeedbackProcessor
    {
        protected:
			const PartID part_id;
            std::queue<Part> pendingScenes;
            Uint16 sequenceSize;
            ros::Duration expectedSceneDuration;
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
            void process_Feedback( const teensy_msgs::FeedbackMsg::ConstPtr &, body_msgs::ProcessedFeedbackMsg & );
            void reset();
            CattyError set_ActionEndReporterMsg( support_msgs::ActionEndReporterMsg & );
            void fillTheRest();
            Uint8 get_scenesLeft() const ;
            ros::Duration get_expectedSceneDuration() const ;
            bool isInAction() const ;
            bool isValid() const ;
            void end_action();
    };
}

#endif
