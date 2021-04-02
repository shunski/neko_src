/*
 * BrushedMotor.cpp implements BrushedMotor.h
 * private member: unsigned char number;
 *                 unsigned int rpm;                      // [0 ~ 127], n = n[rpm]
 *                 unsigned char current;                 // [0 ~ 127], 127 = max_current[A]
 *                 unsigned char current_limit;           // [0 ~ 127], 127 = max_current[A]
 *                 unsigned char pwm;                     // [0 ~ 127]
*/


#include "BrushedMotor.h"


BrushedMotor( Uint8 ID, PartID partID){
    id = ID;
    rpm = set_rpm(0);
    current = set_current(0);
    set_current_limit_ampair( float( default_current ));
    set_pwm(0);
}


void update( Uint8 Rpm, Uint8 Current );
void set( Uint8 Current_limit, Uint8 Pwm );
Uint8 get_number() const { return number; }
Uint8 get_rpm() const { return rpm; }
Uint8 get_current() const { return current; }
Uint8 get_current_limit() const { return current_limit; }
Uint8 get_pwm() const { return pwm; }
void set_rpm( Uint16 Rpm ) { rpm = Rpm; }
void set_current( Uint8 Current ){ current = Current; }
void set_pwm( Uint8 Pwm ) { pwm = Pwm; }
void maximize_current_limit() { current_limit = max_current; }

void set_current_limit( Uint8 Current_limit ){ current_limit = Current_limit; }

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


Uint8 current_float_to_char( float current_float ){
    int current_int = int( current_float * ( 128.0 / max_current ) + 0.5);

    if( current_int < 0 ) { return 0; }

    else if( current_int > 127 ) { return 127; }

    else {
        return current_int;
    }
}

float current_char_to_float( Uint8 current_char ){
    return float( current_char ) / ( 128.0 / max_current );
}


void print() {
    ROS_INFO("Printing Information of DC motor: %d", number);
    ROS_INFO("\t rpm = %d", rpm);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t current_limit = %d", currnt_limnit);
    ROS_INFO("\t pwm = %d", pwm);
}
