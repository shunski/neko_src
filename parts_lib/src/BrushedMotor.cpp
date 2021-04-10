#include <parts_lib/BrushedMotor.h>

BrushedMotor::BrushedMotor( Uint8 ID, PartID partID):
    part_id(partID),
    id(ID),
    rpm(0),
    current(0)
{
	set_pwm(0);
	set_current_limit_ampair( double( default_current ));
}

BrushedMotor::BrushedMotor( const BrushedMotor & original ) : 
	part_id( original.part_id ),
    id( original.id ),
    rpm( original.rpm ),
    current( original.current ),
    pwm( original.pwm ),
    current_limit( current_limit )
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

bool BrushedMotor::set_current_limit_ampair(double Current_limit_ampair) {
    int current_limit_uint8 = current_double_to_uint8( Current_limit_ampair );

    if( current_limit_uint8 > 128 ){
        ROS_INFO("ERROR: in 'set_current_limit_ampair( double )': Too large number received as an argument. Note: max_current=[%d]", max_current);
        return false;
    }

    if( current_limit_uint8 < 0 ) {
        ROS_INFO("ERROR: in 'set_current_limit_ampair( double )': Negative number received as an argument.");
        return false;
    }

	set_current_limit(current_limit_uint8);
    return true;
}


Uint8 BrushedMotor::current_double_to_uint8( double current_double ){
    int current_int = int( current_double * ( 128.0 / max_current ) + 0.5);

    if( current_int < 0 ) { return 0; }
    else if( current_int > 127 ) { return 127; }
    else { return current_int; }
}

double BrushedMotor::current_uint8_to_double( Uint8 current_char ){
    return double( current_char ) / ( 128.0 / max_current );
}

void BrushedMotor::print() {
    ROS_INFO("Printing Information of BrushedMotor: %d", id);
    ROS_INFO("\t rpm = %d", rpm);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t current_limit = %d", current_limit);
    ROS_INFO("\t pwm = %d", pwm);
}

BrushedCommandMsg BrushedMotor::get_CommandMsg() const {
    BrushedCommandMsg msg;
    set_CommandMsg( msg );
    return msg;
}

void BrushedMotor::set_CommandMsg( BrushedCommandMsg & msg ) const {
    msg.id = id;
    msg.current_limit = current_limit;
    msg.pwm = pwm;
}

void BrushedMotor::set( typename BrushedFeedbackMsg::ConstPtr & msg) {
    if( msg->id != id ){
        ROS_INFO("ERROR: Could not set Brushless Motor from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg->id, id);
        return;
    }
    current = msg->current;
    rpm = msg->rpm;
}

void BrushedMotor::set( BrushedFeedbackMsg & msg){
    if( msg.id != id ){
        ROS_INFO("ERROR: Could not set Brushless Motor from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, id);
        return;
    }
    msg.current = current;
    msg.rpm = rpm;
}
