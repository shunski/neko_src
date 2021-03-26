/*
 * BrushedMotor.cpp implements BrushedMotor.h
 * private member: unsigned char number;
 *                 unsigned int rpm;                      // [0 ~ 127], n = n[rpm]
 *                 unsigned char current;                 // [0 ~ 127], 127 = max_current[A]
 *                 unsigned char current_limit;           // [0 ~ 127], 127 = max_current[A]
 *                 unsigned char pwm;                     // [0 ~ 127]
*/


#include "BrushedMotor.h"


BrushedMotor( unsigned char Number ){
    number = Number;
    rpm = set_rpm(0);
    current = set_current(0);
    set_current_limit_ampair( float( default_current ));
    set_pwm(0);
}


void update( unsigned int Rpm, unsigned char Current );
void set( unsigned char Current_limit, unsigned char Pwm );
unsigned char get_number() const { return number; }
unsigned char get_rpm() const { return rpm; }
unsigned char get_current() const { return current; }
unsigned char get_current_limit() const { return current_limit; }
unsigned char get_pwm() const { return pwm; }
void set_rpm( unsigned int Rpm ){ rpm = Rpm; }
void set_current( unsigned char Current ){ current = Current; }
void set_pwm( unsigned char Pwm ) { pwm = Pwm; }
void maximize_current_limit() { current_limit = max_current; }


void set_current_limit( unsigned char Current_limit ){ current_limit = Current_limit; }


bool set_current_limit_ampair(float Current_limit_ampair) {
    if( Current_value_ampair < 0 ) {
	ROS_INFO("ERROR: in 'set_current_limit_ampair( float )': Negative number Received!");
        return false;
    }

    current_limit = set_currentt_limit( unsigned char( current_float_to_char( Current_limit_ampair )));

    if( Current_calue > 128.0 ){
	ROS_INFO("WARNING: in 'set_current_limit_ampair( float )': Too large number Received! Fixed to current_max( %d )", current_max);
        return false;
    }

    return true;
}


unsigned char current_float_to_char( float current_float ){
    int current_int = int( current_float * ( 128.0 / max_current ) + 0.5);

    if( current_int < 0 ) { return 0; }

    else if( current_int > 127 ) { return 127; }

    else {
        return current_int;
    }
}

float current_char_to_float( unsigned char current_char ){
    return float( current_char ) / ( 128.0 / max_current );
}


void print() {
    ROS_INFO("Printing Information of DC motor: %d", number);
    ROS_INFO("\t rpm = %d", rpm);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t current_limit = %d", currnt_limnit);
    ROS_INFO("\t pwm = %d", pwm);
}
