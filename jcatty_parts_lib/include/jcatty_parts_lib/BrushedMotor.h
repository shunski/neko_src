// BrushedMotor.h
#ifndef BRUSHEDMOTOR_H
#define BRUSHEDMOTOR_H

#define max_current = 20
#define default_current = 10;

#include <iostrem>
#include <ros/ros.h>
#include <jcatty_support_msgs/Utilities.h>
#include <jcatty_parts_msgs/BrushedMotorCommandMsg.h>
#include <jcatty_parts_msgs/BrushedMotorFeedbackMsg.h>

typedef jcatty_parts_msgs::BrushedMotorCommandMsg BrushedCommandMsg;
typedef jcatty_parts_msgs::BrushedMotorFeedbackMsg BrushedFeedbackMsg;

class DCmotor
{

    private:
        PartID part_id;
        Uint8 id;
        Uint16 rpm;                    // [0 ~ 255], n = n[rpm] for the sake of storing information. NOT A COMMAND
        Uint8 current;                 // [0 ~ 255], 255 = max_current[A]
        Uint8 current_limit;           // [0 ~ 255], 255 = max_current[A]
        Uint8 pwm;                     // [0 ~ 255]

        bool current_float_to_char( float & current_float );
        void current_char_to_float( Uint8 & current_char );
        void maximize_current_limit();

    public:
        BrushedMotor( Uint8 ID, PartID partID );
        BrushedMotor( Uint8 ID, PartID partID, BrushedCommandMsg & );
        BrushedMotor( Uint8 ID, PartID partID, typename BrushedFeedbackMsg::ConstPtr & );
        BrushedMotor( Uint8 ID, PartID partID, BrushedFeedbackMsg & );
        PartID BrushedMotor::get_part_id();
        Uint8 get_number() const ;
        Uint8 get_rpm() const ;
        Uint8 get_current() const ;
        Uint8 get_current_limit() const ;
        Uint8 get_pwm() const ;

        void set_rpm( Uint16 Rpm );
        void set_current( Uint16 Current );
        bool set_current_ampair( double current_ampair );
        void set_current_limit( Uint8 Current_limit );
        bool set_current_limit_ampair( double Current_limit_ampair );
            // return false if negative value received
            // return false and set limit to max if too large value received;
        void set_pwm( Uint8 Pwm );
        void print();

        BrushedCommandMsg get_CommandMsg() const;
        void set_CommandMsg( BrushedCommandMsg & );
        void set( typename BrushedFeedbackMsg & );
        void set( BrushedFeedbackMsg & );
};


#endif
