#include <actuator_lib/BrushedMotor.h>

BrushedMotor::BrushedMotor( PartID pID, Uint8 ID):
    GenericParts( "BrushedMotor", pID, ID )
{}

BrushedMotor::BrushedMotor( const BrushedMotor & original ) :
    GenericParts( "BrushedMotor", PartID(original.part_id), original.id ),
    current_limit( original.current_limit ),
    pwm( original.pwm ),
    rpm( original.rpm ),
    current( original.current )
{
    state = original.state;
    valid = original.valid;
    well_defined = original.well_defined;
}


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


void BrushedMotor::operator=( const BrushedMotor & original ){
    if( part_id != original.part_id || id != original.id ){
        ROS_INFO("Invalid use of assignment operator for two BrushedMotor objects. Operation failed.");
        return;
    }

    valid = original.valid;
    well_defined = original.well_defined;
    state = original.state;

    current_limit = original.current_limit;
    pwm = original.pwm;

    rpm = original.rpm;
    current = original.current;
}


uint16_t get_rpm() const ;
int16_t get_current_max() const ;

Uint8 get_actual_position() const ;
Uint8 get_current() const ;

const map<ros::Time, int16_t>& get_actual_positions() const ;
const map<ros::Time, int16_t>& get_current_drawed() const ;


void BrushedMotor::print() const
{
    if( valid ){
        ROS_ERROR("Printing information of <%s>: part_id = [%d], id = [%d], \ncurrent_limit\t= [%d], \ncurrent \t= [%d]\n", child_name.c_str(), part_id, id, current_limit );
    }
    else {
        ROS_ERROR("This <%s> is not valid. Please exit the program.", child_name.c_str());
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
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
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
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the command message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
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
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is not suitable for setting the feedback message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
	if ( !valid || !well_defined ){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
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


void BrushedMotor::set_RandomCommandMsg( BrushedCommandMsg & msg ) const {
    msg.part_id = part_id;
    msg.id = id;
    msg.current_limit = 255;
    msg.pwm = rand()%255;
}
