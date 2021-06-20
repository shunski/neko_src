// BrushedMotor.h
#ifndef MOTOR_H
#define MOTOR_H

#define DEFAULT_CURRENT_LIMIT 10 // ampair

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <actuator_lib/Actuator.h>
#include <parts_msgs/MotorMsg.h>
#include <parts_msgs/MotorCommandMsg.h>
#include <parts_msgs/MotorFeedbackMsg.h>

typedef parts_msgs::BrushedMotorMsg BrushedMsg;
typedef parts_msgs::BrushedMotorCommandMsg BrushedCommandMsg;
typedef parts_msgs::BrushedMotorFeedbackMsg BrushedFeedbackMsg;

class Motor : public Actuator<MotorMsg, MotorCommandMsg, MotorFeedbackMsg>
{

    private:
        vector<uint16_t> ideal_position;
        int16_t current_limit;

        uint16_t position;
        uint_8 temperature;
        int16_t current;

        map<ros::Time, uint16_t> actual_positions;
        map<ros::Time, int16_t> current_drawed;

    public:
        BrushedMotor( PartID, Uint8 ID );
        BrushedMotor( const BrushedMotor & );

        BrushedMotor( const BrushedMsg & );
        BrushedMotor( const typename BrushedMsg::ConstPtr & );
        BrushedMotor( const BrushedCommandMsg & );
        BrushedMotor( const typename BrushedCommandMsg::ConstPtr & );
        BrushedMotor( const BrushedFeedbackMsg & );
        BrushedMotor( const typename BrushedFeedbackMsg::ConstPtr & );

        void operator=( const BrushedMotor & );

        uint16_t get_rpm() const ;
        int16_t get_current_limit() const ;

        int16_t get_actual_position() const ;
        uint8_t temperature() const ;
        uint16_t get_current() const ;

        const map<ros::Time, int16_t>& get_actual_positions() const ;
        const map<ros::Time, int16_t>& get_current_drawed() const ;

        void print() const override;

        CattyError set_msg( BrushedMsg & ) const override;
        CattyError set_CommandMsg( BrushedCommandMsg & ) const override;
        CattyError set_FeedbackMsg( BrushedFeedbackMsg & ) const override;

        CattyError set( const BrushedMsg & );
        CattyError set( const typename BrushedMsg::ConstPtr & );
        CattyError set( const BrushedCommandMsg & );
        CattyError set( const typename BrushedCommandMsg::ConstPtr & );
        CattyError set( const BrushedFeedbackMsg & );
        CattyError set( const typename BrushedFeedbackMsg::ConstPtr & );

        void set_RandomCommandMsg( BrushedCommandMsg & ) const ;
};


#endif
