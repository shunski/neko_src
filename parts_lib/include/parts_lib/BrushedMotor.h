// BrushedMotor.h
#ifndef BRUSHEDMOTOR_H
#define BRUSHEDMOTOR_H

#define max_current 20
#define default_current 10

#include <iostream>
#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_msgs/BrushedMotorCommandMsg.h>
#include <parts_msgs/BrushedMotorFeedbackMsg.h>
#include <parts_msgs/BrushedMotorMsg.h>

typedef parts_msgs::BrushedMotorCommandMsg BrushedCommandMsg;
typedef parts_msgs::BrushedMotorFeedbackMsg BrushedFeedbackMsg;

class BrushedMotor
{

    private:
        const PartID part_id;
        const Uint8 id;
        Uint16 rpm;                    // [0 ~ 255], n = n[rpm] for the sake of storing information. NOT A COMMAND
        Uint8 current;                 // [0 ~ 255], 255 = max_current[A]
        Uint8 current_limit;           // [0 ~ 255], 255 = max_current[A]
        Uint8 pwm;                     // [0 ~ 255]

        Uint8 current_double_to_uint8( double current_double );
        double current_uint8_to_double( Uint8 current_uint8 );
        void maximize_current_limit();

    public:
        BrushedMotor( Uint8 ID, PartID );
        BrushedMotor( Uint8 ID, PartID, BrushedCommandMsg & );
        BrushedMotor( Uint8 ID, PartID, typename BrushedFeedbackMsg::ConstPtr & );
        BrushedMotor( Uint8 ID, PartID, BrushedFeedbackMsg & );
		BrushedMotor( const BrushedMotor & );
		BrushedMotor operator=( const BrushedMotor & original ){ return BrushedMotor(original); }
            // explicit assignment definition for the sake of treating const members.
        PartID get_part_id();
        Uint8 get_id() const ;
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
        void print();

        BrushedCommandMsg get_CommandMsg() const;
        void set_CommandMsg( BrushedCommandMsg & ) const ;
        void set( typename BrushedFeedbackMsg::ConstPtr & );
        void set( BrushedFeedbackMsg & );
};


#endif
