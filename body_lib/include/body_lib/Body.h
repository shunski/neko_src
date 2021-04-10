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
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>

#define head_servo_num 1
#define head_brushedMotor_num 0
#define head_brushlessMotor_num 0
#define head_gyroSensor_num 0

#define chest_servo_num 2
#define chest_brushedMotor_num 0
#define chest_brushlessMotor_num 0
#define chest_gyroSensor_num 0

#define waist_servo_num 3
#define waist_brushedMotor_num 0
#define waist_brushlessMotor_num 1
#define waist_gyroSensor_num 0

#define rf_servo_num 1
#define rf_brushedMotor_num 1
#define rf_brushlessMotor_num 0
#define rf_gyroSensor_num 2

#define lf_servo_num 1
#define lf_brushedMotor_num 1
#define lf_brushlessMotor_num 0
#define lf_gyroSensor_num 2

#define rh_servo_num 3
#define rh_brushedMotor_num 1
#define rh_brushlessMotor_num 0
#define rh_gyroSensor_num 2

#define lh_servo_num 3
#define lh_brushedMotor_num 1
#define lh_brushlessMotor_num 0
#define lh_gyroSensor_num 2

namespace Body{
    class Part
    {
        private:
            PartID id;
            std::vector<KondoServo> KondoServoSet;
            std::vector<BrushedMotor> brushedMotorSet;
            std::vector<BrushlessMotor> brushlessMotorSet;
            std::vector<GyroSensor> gyroSensorSet;
			bool isValid;

        public:
			Part ( PartID id );
            Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum );
            void set ( teensy_msgs::FeedbackMsg::ConstPtr & );
            void set ( teensy_msgs::CommandMsg & );
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
            Body() :
                head( HEAD ),
                chest( CHEST ),
				waist( WAIST ),
                right_fore( RFLEG ),
                left_fore( LFLEG ),
                right_hind( RHLEG ),
                left_hind( LHLEG )
            {}
    };

    class MotionController
    {
        protected:
            std::vector<Part> expectedStates;
            Part actualCurrentState;
            std::vector<Part>::iterator expectedCurrentScene;
            const ros::Duration expectedSceneDuration;
            ros::Duration actualSceneDuration;
            ros::Time timeOfActionStart;
            ros::Time timeOfLastAction;

        public:
            MotionController ( PartID );
            void procced();
			void startInitializationAction( initialize_service::PartInitialization::Request &,
											initialize_servoce::PartInitialization::Response & );
			void startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & );
            void set_CommandMsg( teensy_msgs::CommandMsg & msg );
            bool isEnd() { return expectedCurrentScene == expectedStates.end(); }
            void set_feedbackMsg ( motioncontroll_action::MotionControllFeedback & feedbackMsg ) {feedbackMsg = this->get_feedbackMsg(); }
            void set_resultMsg ( motioncontroll_action::MotionControllResult & resultMsg ) { resultMsg = this->get_resultMsg(); }
            motioncontroll_action::MotionControllFeedback get_feedbackMsg() const ;
            motioncontroll_action::MotionControllResult get_resultMsg() const ;
            ros::Duration get_actualSceneDuration() { return actualSceneDuration; }
    };

    class FeedbackProcessor
    {
        protected:
            Part currentState;

        public:
            FeedbackProcessor( PartID );
            void set( teensy_msgs::FeedbackMsg::ConstPtr & msg );
            Part processFeedback( teensy_msgs::FeedbackMsg::ConstPtr & msg );
    };

	class PartInitializer
	{
		protected:
			partID id;
		public:
			initializePosition();
	};
}

#endif
