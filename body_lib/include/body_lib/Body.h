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
#include <parts_lib/CattyParts.h>
#include <body_msgs/PartCommandMsg.h>
#include <body_msgs/PartMsg.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>
//#include <support_srvs/LocomotionActionStateSrv.h> //これやってもsupport_srvsインクルードできてなさそうだった->138行目

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
            Part( PartID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum );
            Part( PartID, const std::vector<KondoServo>&,
                          const std::vector<BrushedMotor>&,
                          const std::vector<BrushlessMotor>& );
			Part( PartID, const std::vector<parts_msgs::KondoServoCommandMsg> &,
                          const std::vector<parts_msgs::BrushedMotorCommandMsg> &,
                          const std::vector<parts_msgs::BrushlessMotorCommandMsg> & );
            CattyError set( const teensy_msgs::FeedbackMsg::ConstPtr & );
            CattyError set( const teensy_msgs::CommandMsg::ConstPtr & );

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
            std::vector<Part> expectedStates;
            Part actualCurrentState;
            std::vector<Part>::iterator expectedCurrentState;
            const ros::Duration expectedSceneDuration;
            ros::Duration actualCurrentSceneDuration;
            ros::Time timeOfActionStart;
            ros::Time timeOfLastAction;
			bool valid;

        public:
            MotionController ( PartID );
            CattyError set_action( motioncontroll_action::MotionControllGoal::ConstPtr & );
            void procced();
			// void startInitializationAction( initialize_service::PartInitialization::Request &,
			//								initialize_servoce::PartInitialization::Response & );
			void startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & );
            CattyError set_CommandMsg( teensy_msgs::CommandMsg & ) const ;
            bool isEnd() const ;
            CattyError set_feedbackMsg( motioncontroll_action::MotionControllFeedback & );
            CattyError set_resultMsg( motioncontroll_action::MotionControllResult & );
            ros::Duration get_actualCurrentSceneDuration() const;
			ros::Duration get_expectedSceneDuration() const;
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
            void start_action( Uint16 SequenceSize, std::string & );
            void add_pendingScenes( const teensy_msgs::CommandMsg::ConstPtr & ); // When command is received from MotionController
            Part process_Feedback( const teensy_msgs::FeedbackMsg::ConstPtr & ); // When feedback is received from teensy
            void reset_processing();
            //CattyError set_stateOfScene( support_srvs::LocmotionActionStateSrv::Response & );   //support_srvsがincludeできてないっぽい
            void fillTheRest();
            bool isInAction() const ;
            bool isValid() const ;
    };
}

#endif
