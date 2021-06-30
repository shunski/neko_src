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
            const PartId part_id;
            uint16_t scene_id;

			bool valid;
			bool well_defined;
			ObjectState state;

        public:
            Body();
		    Body( PartId );
            Body( PartId, uint16_t SceneID );
            Body( const teensy_msgs::CommandMsg::ConstPtr & );
            Body( const teensy_msgs::FeedbackMsg::ConstPtr & );
            Body( PartId, ObjectState, PartProperties );
            Body( const Part & );

            void operator=( const Part & );

            PartId get_part_id() const ;
            uint16_t get_scene_id() const ;
            ObjectState get_state() const ;

            CattyError set( const teensy_msgs::FeedbackMsg::ConstPtr & );
            CattyError set( const teensy_msgs::CommandMsg::ConstPtr & );
			CattyError set( const body_msgs::PartMsg::ConstPtr & );

            CattyError set_part_msg( body_msgs::PartMsg & );
            CattyError set_part_command_msg( body_msgs::PartCommandMsg & );
			CattyError set_command_msg( teensy_msgs::CommandMsg & );

			bool is_valid() const ;
			bool is_well_defined() const ;

            void set_random_part_command_msg( body_msgs::PartCommandMsg & );
    };


    class MotionController
    {
        protected:
			const PartId part_id;
			uint16_t sequence_size;
			uint16_t current_scene_id_processed;                     // keeping track of how many scenes are processed
            std::vector<Part> expected_states;
            Part actual_current_state;
            std::vector<Part>::iterator expected_current_state;

			std::queue<motioncontroll_action::MotionControllFeedback> motion_controll_feedbacks;
			motioncontroll_action::MotionControllResult motion_controll_result;

            ros::Duration expected_scene_duration;
            ros::Time time_of_action_start;

            bool in_action;
			bool valid;

        public:
            MotionController( PartId );
            CattyError set_action( const motioncontroll_action::MotionControllGoal::ConstPtr & );
            void proceed();
			void start_motioncontroll_action();
            CattyError set_command_msg( teensy_msgs::CommandMsg & ) const ;
            bool is_end() const ;
            void initialize_feedbackMsg();
            void initialize_resultMsg();
            CattyError update_locomotion_action_feedback_msg( const body_msgs::ProcessedFeedbackMsg::ConstPtr &, const std::string & nodeName );
            CattyError update_locomotion_action_result_msg( const support_msgs::ActionEndReporterMsg::ConstPtr &, const std::string & );
			bool set_locomotion_action_feedback_msg( motioncontroll_action::MotionControllFeedback & );
			motioncontroll_action::MotionControllResult get_locomotion_action_result_msg() const;
			ros::Duration get_expected_scene_duration() const;
            CattyError set_action_start_notifier( support_msgs::ActionStartNotifierMsg & );
            void end_action();
            void reset();
            bool is_in_action() const ;
			bool is_valid() const;
    };



    class FeedbackProcessor
    {
        protected:
			const PartId part_id;
            std::queue<Part> pending_scenes;
            uint16_t sequence_size;
            ros::Duration expected_scene_duration;
            uint16_t current_scene_id_received;
            uint16_t current_scene_id_processed;
            uint16_t num_missing_expected_scenes;              // # of scenes heard from teensy but not from MotionController
            uint16_t num_missing_actual_scenes;                // # of scenes heard from MotionController but not from teensy
            uint16_t num_totally_missing_scenes;               // # of scenes not processed at all which were supposed to be there (guessed by scene_id of messages)
            std::vector<ObjectState> state_of_scenes;
            bool in_action;
            bool valid;

        public:
            FeedbackProcessor( PartId );
            CattyError start_action( const support_msgs::ActionStartNotifierMsg::ConstPtr &, const std::string & NodeName );
            bool add_pendingScenes( const teensy_msgs::CommandMsg::ConstPtr & ); // When command is received from MotionController. return true if the action ends.
            void process_feedback( const teensy_msgs::FeedbackMsg::ConstPtr &, body_msgs::ProcessedFeedbackMsg & );
            void reset();
            CattyError set_action_end_reporter_msg( support_msgs::ActionEndReporterMsg & );
            void fill_the_rest();
            uint8_t get_scenesLeft() const ;
            ros::Duration get_expected_scene_duration() const ;
            bool is_in_action() const ;
            bool is_valid() const ;
            void end_action();
    };
}

#endif
