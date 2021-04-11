// Body.h

#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <motioncontroll_action/MotionControllAction.h>
#include <motioncontroll_action/MotionControllGoal.h>
#include <motioncontroll_action/MotionControllResult.h>
#include <motioncontroll_action/MotionControllFeedback.h>
#include <support_msgs/CalibrationMsg.h>
#include <parts_lib/Parts.h>
#include <body_msgs/PartCommandMsg.h>
#include <body_msgs/PartMsg.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>


namespace Body{
    class Part
    {
        private:
            PartID id;
            std::vector<KondoServo> kondoServoSet;
            std::vector<BrushedMotor> brushedMotorSet;
            std::vector<BrushlessMotor> brushlessMotorSet;
            std::vector<GyroSensor> gyroSensorSet;
			bool valid;

        public:
            Part();
		    Part( PartID ID );
            Part( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum );
            Part( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum );
            void set( teensy_msgs::FeedbackMsg::ConstPtr & );
            void set( teensy_msgs::CommandMsg & );
            void set_CommandMsg( teensy_msgs::CommandMsg & );
            body_msgs::PartMsg get_PartMsg() const ;
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
			PartID id;
            std::vector<Part> expectedStates;
            Part actualCurrentState;
            std::vector<Part>::iterator expectedCurrentState;
            const ros::Duration expectedSceneDuration;                                             // 変数みたいな使い方はできない？
            ros::Duration actualCurrentSceneDuration;                                              // ..
            ros::Time timeOfActionStart;                                                           // ..
            ros::Time timeOfLastAction;                                                            // ..
			bool valid;

        public:
            MotionController ( PartID );
            CattyError set_action( body_msgs::PartCommandMsg::ConstPtr & );
            void procced();
			// void startInitializationAction( initialize_service::PartInitialization::Request &,
			//								initialize_servoce::PartInitialization::Response & );
			void startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & );
            void set_CommandMsg( teensy_msgs::CommandMsg & ) const ;
            bool isEnd() const ;
            void set_feedbackMsg( motioncontroll_action::MotionControllFeedback & ) const ;
            void set_resultMsg( motioncontroll_action::MotionControllResult & ) const ;
            motioncontroll_action::MotionControllFeedback get_feedbackMsg() const ;
            motioncontroll_action::MotionControllResult get_resultMsg() const ;
            ros::Duration get_actualCurrentSceneDuration() const;
			ros::Duration get_expectedSceneDuration() const;
			bool isValid();

    };

    class FeedbackProcessor
    {
        protected:
            Part currentState;
            Part previousState;

        public:
            FeedbackProcessor( PartID );
            void set( teensy_msgs::FeedbackMsg::ConstPtr & );
            Part processFeedback( teensy_msgs::FeedbackMsg::ConstPtr & );
    };
}

#endif
