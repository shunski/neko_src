// BrushedMotor.h
#ifndef BRUSHEDMOTOR_H
#define BRUSHEDMOTOR_H

#define max_current 20
#define default_current 10

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_lib/GenericParts.h>
#include <parts_msgs/BrushedMotorMsg.h>
#include <parts_msgs/BrushedMotorCommandMsg.h>
#include <parts_msgs/BrushedMotorFeedbackMsg.h>

typedef parts_msgs::BrushedMotorMsg BrushedMsg;
typedef parts_msgs::BrushedMotorCommandMsg BrushedCommandMsg;
typedef parts_msgs::BrushedMotorFeedbackMsg BrushedFeedbackMsg;

class BrushedMotor : public GenericParts<BrushedMsg, BrushedCommandMsg, BrushedFeedbackMsg>
{

    private:
        Uint8 current_limit;           // [0 ~ 255], 255 = max_current[A]
        Uint8 pwm;                     // [0 ~ 255]

        Uint16 rpm;                    // [0 ~ 255], n = n[rpm]
        Uint8 current;                 // [0 ~ 255], 255 = max_current[A]

        Uint8 current_double_to_uint8( double current_double );
        double current_uint8_to_double( Uint8 current_uint8 );
        void maximize_current_limit();

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

        Uint8 get_rpm() const ;
        Uint8 get_current() const ;
        Uint8 get_current_limit() const ;
        Uint8 get_pwm() const ;

        void set_rpm( Uint16 Rpm );
        void set_current( Uint8 Current );
        bool set_current_ampair( double current_ampair );
        void set_current_limit( Uint8 Current_limit );
        bool set_current_limit_ampair( double Current_limit_ampair );
            // return false if negative value received
            // return false and set limit to max if too large value received;
        void set_pwm( Uint8 Pwm );

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
