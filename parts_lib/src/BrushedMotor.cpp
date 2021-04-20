#include <parts_lib/BrushedMotor.h>

BrushedMotor::BrushedMotor( PartID pID, Uint8 ID):
    GenericParts( "BrushedMotor", pID, ID )
{}

BrushedMotor::BrushedMotor( const BrushedMotor & original ) :
    GenericParts( "BrushedMotor", PartID(original.part_id), original.id ),
    current_limit( original.current_limit ),
    pwm( original.pwm ),
    rpm( original.rpm ),
    current( original.current )
{}


BrushedMotor::BrushedMotor( const BrushedMsg & msg ):
    GenericParts( "BrushedMotor", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


BrushedMotor::BrushedMotor( const typename BrushedMsg::ConstPtr & msg ):
    GenericParts( "BrushedMotor", PartID(msg->part_id), msg->id )
{ this->set( msg ); }


BrushedMotor::BrushedMotor( const BrushedCommandMsg & msg ):
    GenericParts( "BrushedMotor", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


BrushedMotor::BrushedMotor( const typename BrushedCommandMsg::ConstPtr & msg ):
    GenericParts( "BrushedMotor", PartID(msg->part_id), msg->id )
{ this->set( msg ); }


BrushedMotor::BrushedMotor( const BrushedFeedbackMsg & msg ):
    GenericParts( "BrushedMotor", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


BrushedMotor::BrushedMotor( const typename BrushedFeedbackMsg::ConstPtr & msg ):
    GenericParts( "BrushedMotor", PartID(msg->part_id), msg->id )
{ this->set( msg ); }


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


Uint8 current_limit;
Uint8 pwm;
Uint16 rpm;
Uint8 current;


void BrushedMotor::print() const
{
    if( valid ){
        ROS_INFO("Printing Information of <%s>: part_id = [%d], id = [%d], \ncurrent_limit\t= [%d], \npwm \t= [%d], \nrpm \t= [%d], \ncurrent \t= [%d], \n", child_name.c_str(), part_id, id, current_limit, pwm, rpm, current );
    }
    else {
        ROS_INFO("This <%s> is not valid. Please exit the program.", child_name.c_str());
    }
}


CattyError BrushedMotor::set_msg( BrushedMsg & msg ) const
{
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg.part_id = part_id;
		msg.id = id;
	} else {
		CattyError error = check_msg_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error( msg );
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error( msg );
			return error;
		}
	}
	if ( state != GENERAL ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	msg.current_limit = current_limit;
	msg.pwm = pwm;

	msg.rpm = rpm;
    msg.current = current;

    return SUCCESS;
}


CattyError BrushedMotor::set_CommandMsg( BrushedCommandMsg & msg ) const {
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg.part_id = part_id;
		msg.id = id;
	} else {
		CattyError error = check_msg_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error( msg );
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error( msg );
			return error;
		}
	}
	if ( !( state == COMMAND || state == GENERAL )) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

    msg.current_limit = current_limit;
	msg.pwm = pwm;

    return SUCCESS;
}


CattyError BrushedMotor::set_FeedbackMsg( BrushedFeedbackMsg & msg ) const {
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg.part_id = part_id;
		msg.id = id;
	} else {
		CattyError error = check_msg_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error( msg );
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error( msg );
			return error;
		}
	}
	if ( !( state == FEEDBACK || state == GENERAL )) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

    msg.rpm = rpm;
    msg.current = current;

    return SUCCESS;
}


CattyError BrushedMotor::set( const BrushedMsg & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    state = GENERAL;

    current_limit = msg.current_limit;
	pwm = msg.pwm;

	rpm = msg.rpm;
    current = msg.current;

	well_defined = true;

    return SUCCESS;
}


CattyError BrushedMotor::set( const typename BrushedMsg::ConstPtr & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    state = GENERAL;

    current_limit = msg->current_limit;
	pwm = msg->pwm;

	rpm = msg->rpm;
    current = msg->current;

	well_defined = true;

    return SUCCESS;
}


CattyError BrushedMotor::set( const BrushedCommandMsg & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    if ( state == INVALID ) state = COMMAND;
    else if ( state == FEEDBACK ) state = GENERAL;

    current_limit = msg.current_limit;
	pwm = msg.pwm;

	well_defined = true;

    return SUCCESS;
}


CattyError BrushedMotor::set( const BrushedCommandMsg::ConstPtr & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    if ( state == INVALID ) state = COMMAND;
    else if ( state == FEEDBACK ) state = GENERAL;

    current_limit = msg->current_limit;
	pwm = msg->pwm;

	well_defined = true;

    return SUCCESS;
}


CattyError BrushedMotor::set( const BrushedFeedbackMsg & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    if ( state == INVALID ) state = FEEDBACK;
    else if ( state == COMMAND ) state = GENERAL;

    rpm = msg.rpm;
    current = msg.current;

	well_defined = true;

    return SUCCESS;
}


CattyError BrushedMotor::set( const typename BrushedFeedbackMsg::ConstPtr & msg )
{
	CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    if ( state == INVALID ) state = FEEDBACK;
    else if ( state == COMMAND ) state = GENERAL;

    rpm = msg->rpm;
    current = msg->current;

	well_defined = true;

    return SUCCESS;
}
