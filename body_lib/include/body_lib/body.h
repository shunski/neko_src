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
}

#endif
