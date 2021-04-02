#include <jcatty_parts_lib/BrushedMotor.h>

BrushedMotor::BrushedMotor( Uint8 ID, PartID partID):
    part_id(partID),
    id(ID),
    rpm(0),
    current(0),
    set_current_limit_ampair( float( default_current )),
    set_pwm(0)
{}

PartID BrushedMotor::get_part_id() { return part_id; }
Uint8 BrushedMotor::get_id() const { return id; }
Uint8 BrushedMotor::get_rpm() const { return rpm; }
Uint8 BrushedMotor::get_current() const { return current; }
Uint8 BrushedMotor::get_current_limit() const { return current_limit; }
Uint8 BrushedMotor::get_pwm() const { return pwm; }
void BrushedMotor::set_rpm( Uint16 Rpm ) { rpm = Rpm; }
void BrushedMotor::set_current( Uint8 Current ){ current = Current; }
void BrushedMotor::set_pwm( Uint8 Pwm ) { pwm = Pwm; }
void BrushedMotor::maximize_current_limit() { current_limit = max_current; }

void BrushedMotor::set_current_limit( Uint8 Current_limit ){ current_limit = Current_limit; }

bool BrushedMotor::set_current_limit_ampair(float Current_limit_ampair) {
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


Uint8 BrushedMotor::current_float_to_char( float current_float ){
    int current_int = int( current_float * ( 128.0 / max_current ) + 0.5);

    if( current_int < 0 ) { return 0; }
    else if( current_int > 127 ) { return 127; }
    else { return current_int; }
}

float BrushedMotor::current_char_to_float( Uint8 current_char ){
    return float( current_char ) / ( 128.0 / max_current );
}

void BrushedMotor::print() {
    ROS_INFO("Printing Information of DC motor: %d", number);
    ROS_INFO("\t rpm = %d", rpm);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t current_limit = %d", currnt_limnit);
    ROS_INFO("\t pwm = %d", pwm);
}

BrushedCommandMsg BrushedMotor::get_CommandMsg() const {
    BrushedCommandMsg msg;
    set_CommandMsg(msg);
    return msg;
}

void BrushedMotor::set_CommandMsg( BrushedCommandMsg & msg ) {
    msg.id = id;
    msg.current_limit = limit;
    msg.pwm = pwm;
}

void BrushedMotor::set( typename BrushedFeedbackMsg & ) {
    if( msg->id != id ){
        ROS_INFO("ERROR: Could not set Brushless Motor from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg->id, id);
        return;
    }
    msg->current = current;
    msg->rpm = rpm;
}

void BrushedMotor::set( BrushedFeedbackMsg & ){
    if( msg.id != id ){
        ROS_INFO("ERROR: Could not set Brushless Motor from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, id);
        return;
    }
    msg.current = current;
    msg.rpm = rpm;
}
