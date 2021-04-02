// McNode.h

#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <actionlib/client/simple_action_server.h>
#include <jcatty_action/MotionControllAction.h>
#include <jcatty_action/MotionControllGoal.h>
#include <jcatty_action/MotionControllResult.h>
#include <jcatty_action/MotionControllFeedback.h>
#include <jcatty_support_msgs/CalibrationMsg.h>
#include <jcatty_body_lib/Part.h>
#include <jcatty_body_msgs/PartCmdMsg.h>
#include <jcatty_teensy_msgs/CommandMsg.h>
#include <jcatty_teensy_msgs/InfoMsg.h>

#define head_servo_num 1;
#define head_brushedMotor_num 0;
#define head_brushlessMotor_num 0;
#define head_gyroSensor_num 0;

#define chest_servo_num 2;
#define chest_brushedMotor_num 0;
#define chest_brushlessMotor_num 0;
#define chest_gyroSensor_num 0;

#define waist_servo_num 3;
#define waist_brushedMotor_num 0;
#define waist_brushlessMotor_num 1;
#define waist_gyroSensor_num 0;

#define rf_servo_num 1;
#define rf_brushedMotor_num 1;
#define rf_brushlessMotor_num 0;
#define rf_gyroSensor_num 2;

#define lf_servo_num 1;
#define lf_brushedMotor_num 1;
#define lf_brushlessMotor_num 0;
#define lf_gyroSensor_num 2;

#define rh_servo_num 3;
#define rh_brushedMotor_num 1;
#define rh_brushlessMotor_num 0;
#define rh_gyroSensor_num 2;

#define lh_servo_num 3;
#define rh_brushedMotor_num 1;
#define rh_brushlessMotor_num 0;
#define rh_gyroSensor_num 2;

namespace Body{
    class Part
    {
        private:
            std::string partName;
            std::vector<KondoServo> servoSet;
            std::vector<BrushedMotor> brushedMotorSet;
            std::vector<BrushlessMotor> brushlessMotorSet;
            std::vector<GyroSensor> gyroSensorSet;

        public:
            Part ( std::string PartName, unsigned char servoNum, unsigned char brushedMotorNum, unsigned char brushlessMotorNum, unsigned char gyroSensorNum );
            void set ( jcatty_teensy_msgs::InfoMsg::ConstPtr & );
            void set ( jcatty_teensy_msgs::CommandMsg );
            void set_CommandMsg( jcatty_teensy_msgs::CommandMsg );
            jcatty_body_msgs::PartMsg get_PartMsg() const ;
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
                head("Head", head_servo_num, head_brushedMotor_num, head_brushlessMotor_num, head_gyroSensor_num, head_encoder_num),
                head("Chest", ), // fix
                head("Waist", ), // fix
                head("RightFore", ), // fix
                head("LeftFore", ), // fix
                head("RightHind", ), // fix
                head("LeftHind", ) // fix
            {}
    };

    class MotionController
    {
        protected:
            vector<Part> expectedStates;
            Part actualState;
            vector<Part>::iterator currentScene;
            const ros::Duration expectedSceneDuration;
            ros::Duration actualSceneDuration;
            ros::Time timeOfActionStart;
            ros::Time timeOfLastAction;

        public:
            MotionController ( Part part, jcatty_body_msgs::PartCmdMsg::ConstPtr & msg ) { this->set ( part, msg ); }
            void set ( Part part, jcatty_body_msgs::PartCmdMsg::ConstPtr & msg ) { this->set_sequence ( msg ); }
            void set_sequence ( jcatty_body_msgs::PartCmdMsg::ConstPtr & msg );
            void initializePosition();
            void procced();
            void startAction() { timeOfActionStart = ros::Time::now(); }
            void set_CommandMsg( jcattty_teensy_msgs::HindlegCommandMsg & msg );
            bool isEnd() { return currrentScene == expectedStates.end(); }
            void set_feedbackMsg ( jcatty_locomotion_action::MotionControllFeedback & feedbackMsg ) {feedbackMsg = this->get_feedbackMsg(); }
            void set_resultMsg ( jcatty_locomotion_action::MotionControllResult & resultMsg ) { resultMsg = this->get_resultMsg(); }
            jcatty_locomotion_action::MotionControllFeedback get_feedbackMsg() const ;
            jcatty_locomotion_action::MotionControllResult get_resultMsg() const ;
            ros::Duration get_sceneDuration() { return sceneDuration; }
    };

    class FeedbackProcessor
    {
        protected:
            Part currentState;
            Part previousState;

        public:
            FeedbackProcessor( jcatty_teensy_msgs::InfoMsg::ConstPtr & msg ) { this->set( msg ); }
            void set( jcatty_teensy_msgs::InfoMsg::ConstPtr & msg ) { currentState.set( msg ); }
            Part processFeedback( InfoMsg::ConstPtr & msg ) {
                previousState = currentState;
                currentState.set( msg );
            }
    };
}

#endif
