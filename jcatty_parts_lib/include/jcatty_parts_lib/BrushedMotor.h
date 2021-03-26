// DCmotor.cpp implemented by DCmotor.cpp


#ifndef BRUSHEDMOTOR_H
#define BRUSHEDMOTOR_H


#define max_current = 20
#define default_currrent = 10;


#include <iostrem>
#include <ros/ros.h>
#include <jcatty_parts_msgs/BrushedMotorMsg.h>


class DCmotor
{

    private:
        unsigned char number;
        unsigned int rpm;                      // [0 ~ 127], n = n[rpm]
        unsigned char current;                 // [0 ~ 127], 127 = max_current[A]
        unsigned char current_limit;           // [0 ~ 127], 127 = max_current[A]
        unsigned char pwm;                     // [0 ~ 127]

        bool current_float_to_char( float & current_float );
        void current_char_to_float( unsigned char & current_char );
        void maximize_current_limit();

    public:
        DCmotor(unsigned char Number);
        void update( unsigned int Rpm, unsigned char Current );
        void set( unsigned char Current_limit, unsigned char Pwm );
        unsigned char get_number() const ;
        unsigned char get_rpm() const ;
        unsigned char get_current() const ;
        unsigned char get_current_limit() const ;
        unsigned char get_pwm() const ;
        void set_rpm( unsigned int Rpm );
        void set_current( unsigned char Current );
        bool set_current_ampair( current_ampair );
        void set_current_limit( unsigned char Current_limit );
        bool set_current_limit_ampair( float Current_limit_ampair );
            // return false if negative value received
            // return false and set limit to max if too large value received;
        void set_pwm( unsigned char Pwm );
        void print();
};


#endif
