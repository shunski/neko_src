// Body.h

#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>
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


namespace Body{
    class Part
    {
        private:
            const PartID part_id;
            std::vector<KondoServo> kondoServoSet;
            std::vector<BrushedMotor> brushedMotorSet;
            std::vector<BrushlessMotor> brushlessMotorSet;
            std::vector<MotionSensor> motionSensorSet;
			bool valid;

        public:
            Part();
		    Part( PartID );
            Part( PartID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum );
            Part( PartID, std::vector<KondoServo>     KondoServoSet,
                          std::vector<BrushedMotor>   BrushedMotorSet,
                          std::vector<BrushlessMotor> BrushlessMotorSet );
            CattyError set( const teensy_msgs::FeedbackMsg::ConstPtr & );
            CattyError set( const teensy_msgs::CommandMsg::ConstPtr & );
            void set_CommandMsg( teensy_msgs::CommandMsg & );
            body_msgs::PartMsg get_PartMsg() const ;
			void set_PartMsg( body_msgs::PartMsg & );
			bool isValid();
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
            CattyError set_feedbackMsg( motioncontroll_action::MotionControllFeedback & ) const ;
            CattyError set_resultMsg( motioncontroll_action::MotionControllResult & ) const ;
            motioncontroll_action::MotionControllFeedback generate_feedbackMsg() const ;
            motioncontroll_action::MotionControllResult generate_resultMsg() const ;
            ros::Duration get_actualCurrentSceneDuration() const;
			ros::Duration get_expectedSceneDuration() const;
			bool isValid();

    };

    class FeedbackProcessor
    {
        protected:
			const PartID part_id;
            Part currentState;
            Part previousState;

        public:
            FeedbackProcessor( PartID );
            void set( teensy_msgs::FeedbackMsg::ConstPtr & );
            Part processFeedback( teensy_msgs::FeedbackMsg::ConstPtr & );
    };
}

#endif
