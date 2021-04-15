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

        Uint16 rpm;                    // [0 ~ 255], n = n[rpm] for the sake of storing information. NOT A COMMAND
        Uint8 current;                 // [0 ~ 255], 255 = max_current[A]

        Uint8 current_double_to_uint8( double current_double );
        double current_uint8_to_double( Uint8 current_uint8 );
        void maximize_current_limit();

    public:
        BrushedMotor( Uint8 ID, PartID );
        BrushedMotor( const BrushedMotor & );

        BrushedMotor( const BrushedMsg & );
        BrushedMotor( const typename BrushedMsg::ConstPtr & );
        BrushedMotor( const BrushedCommandMsg & );
        BrushedMotor( const typename BrushedCommandMsg::ConstPtr & );
        BrushedMotor( const BrushedFeedbackMsg & );
        BrushedMotor( const typename BrushedFeedbackMsg::ConstPtr & );

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

        CattyPartsError set_msg() const override;
        CattyPartsError set_CommandMsg( BrushedCommandMsg & ) const override;
        CattyPartsError set_FeedbackMsg( BrushedFeedbackMsg & ) const override;

        CattyPartsError set( const BrushedMsg & );
        CattyPartsError set( const typename BrushedMsg::ConstPtr & );
        CattyPartsError set( const BrushedVCommandMsg & );
        CattyPartsError set( const typename BrushedCommandMsg::ConstPtr & );
        CattyPartsError set( const BrushedFeedbackMsg & );
        CattyPartsError set( const typename BrushedFeedbackMsg::ConstPtr & );
};


#endif
