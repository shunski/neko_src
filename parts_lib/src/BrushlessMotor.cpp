#include <parts_lib/BrushlessMotor.h>

BrushlessMotor::BrushlessMotor( PartID pID, Uint8 ID ):
	GenrericParts( "BrushlessMotor", pID, ID )
{}


BrushlessMotor::BrushlessMotor( const BrushlessMotor & original ):
	GenrericParts( "BrushlessMotor", original.part_id, original.id ),
	voltage( original.voltage ),
	position( original.position ),
	speed( original.speed ),
	current( original.current )
{
	state = original.state;
	valid = original.valid;
	well_defined = riiginal.well_defined;
}


BrushlessMotor::BrushlessMotor( const BrushlessMsg & msg ):
	GenrericParts( "BrushlessMotor", msg.part_id, msg.id )
{ this->set( msg ); }


BrushlessMotor::BrushlessMotor( const typename BrushlessMsg::ConstPtr & msg ):
	GenrericParts( "BrushlessMotor", msg->part_id, msg->id )
{ this->set( msg ); }


BrushlessMotor::BrushlessMotor( const BrushlessCommandMsg & msg ):
	GenrericParts( "BrushlessMotor", msg.part_id, msg.id )
{ this->set( msg ); }


BrushlessMotor::BrushlessMotor( const typename BrushlessCommandMsg::ConstPtr & msg ):
	GenrericParts( "BrushlessMotor", msg->part_id, msg->id )
{ this->set( msg ); }


BrushlessMotor::BrushlessMotor( const BrushlessFeedbackMsg & msg ):
	GenrericParts( "BrushlessMotor", msg.part_id, msg.id )
{ this->set( msg ); }


BrushlessMotor::BrushlessMotor( typename BrushlessFeedbackMsg::ConstPtr & msg ):
	GenrericParts( "BrushlessMotor", msg->part_id, msg->id )
{ this->set( msg ); }


PartID BrushlessMotor::get_part_id() const { return part_id; }
Uint8 BrushlessMotor::get_id() const { return id }
Int16 BrushlessMotor::get_voltage() const { return voltage; }
Uint16 BrushlessMotor::get_position() const { return position; }
Int16 BrushlessMotor::get_speed() const { return speed; }
Uint16 BrushlessMotor::get_current() const { return current; }

CattyPartsError BrushlessMotor::set_voltage( Int16 Voltage) {
	if ( Voltage > 30000 ) {
		voltage =  30000;
		ROS_INFO("set_voltage(Int16):Could not set the value of argument. Value was too big.");
		return WARNING;
	}
	if ( Voltage < -30000 ) {
		voltage =  -30000;
		ROS_INFO("set_voltage(Int16):Could not set the value of argument. Value was too small.");
		return WARNING;
	}
	voltage = Voltage;
	return SUCCESS;
}


CattyPartsError BrushlessMotor::set_position( Uint16 Position ) {
	if ( Position > 8191 ) {
		position =  8191;
		ROS_INFO("set_position(Uint16):Could not set the value of argument. Value was too big.");
		return WARNING;
	}
	if ( Position < 0 ) {
		position =  0;
		ROS_INFO("set_position(Uint16):Could not set the value of argument. Value was too small.");
		return WARNING;
	}
	position = Position;
	return SUCCESS;
}


CattyPartsError BrushlessMotor::set_speed( Int16 Speed) {
	speed = Speed;
	return SUCCESS;
}


CattyPartsError BrushlessMotor::set_current( Uint16 Current ) {
	current = Current;
	return SUCCESS;
}


void BrushlessMotor::print() const override {
	if( valid ){
		ROS_INFO("Printing Information of <%s>: part_id = [%d], id = [%d], \n
		voltage \t= [%d], \n
		position \t= [%d], \n
		speed \t= [%d], \n
		current \t= [%d] \n\n",
		child_name.c_str(), part_id, id, voltage, position, speed, current );
	}
	else {
		ROS_INFO("This <%s> is not valid. Please exit the program.", child_name.c_str());
	}
}


CattyPartsError BrushlessMotor::set_msg( BrushlessMsg & msg ) const override {
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg->part_id = part_id;
		msg->id = id;
	} else {
		CattyPartsError error = check_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error();
			valid = false;
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error();
			valid = false;
			return error;
		}
	}
	if ( state != GENERAL ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.voltage = voltage;
	msg.position = position;
	msg.speed = speed;
	msg.current = current;

    return SUCCESS;
}


CattyPartsError BrushlessMottor::set_CommandMsg( BrushlessCommadMsg & msg ) const override {
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg->part_id = part_id;
		msg->id = id;
	} else {
		CattyPartsError error = check_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error();
			valid = false;
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error();
			valid = false;
			return error;
		}
	}
	if ( !( state == COMMAND || state == GENERAL )) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the command message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a command message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.voltage = voltage;

    return SUCCESS;
}


CattyPartsError BrushlessMotor::set_FeedbackMsg( BrushlessFeedbackMsg & msg ) const override {
	if ( msg.part_id == 0 && msg.id == 0 ) {
		msg->part_id = part_id;
		msg->id = id;
	} else {
		CattyPartsError error = check_id( msg );
		if ( error == PART_ID_NOT_MATCH ) {
			print_msg_part_id_error();
			valid = false;
			return error;
		} else if ( error == ID_NOT_MATCH ) {
			print_msg_id_error();
			valid = false;
			return error;
		}
	}
	if ( !( state == FEEDBACK || state == GENERAL )) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the feedback message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a feedback message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.position = position;
	msg.speed = speed;
	msg.current = current;

    return SUCCESS;
}



CattyPartsError BrushlessMotor::set( const BrushlessMsg & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	state = GENERAL;

	voltage = msg.voltage;
	position = msg.position;
	speed = msg.speed;
	current = msg.current;

	well_defined = true;

	return SUCCESS;
}


CattyPartsError BrushlessMotor::set( const typename BrushlessMsg::ConstPtr & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	state = GENERAL;

	voltage = msg->voltage;
	position = msg->position;
	speed = msg->speed;
	current = msg->current;

	well_defined = true;

	return SUCCESS;
}


CattyPartsError BrushlessMotor::set( const BrushlessCommandMsg & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	else if ( state == FEEDBACK ) state = GENERAL;

	voltage = msg.voltage;

	well_defined = true;

	return SUCCESS;
}


CattyPartsError BrushlessMotor::set( const typename BrushlessCommandMsg::ConstPtr & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	else if ( state == FEEDBACK ) state = GENERAL;

	voltage = msg->voltage;

	well_defined = true;

	return SUCCESS;
}


CattyPartsError BrushlessMotor::set( const BrushlessFeedbackMsg & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	if ( state == NONE ) state = FEEDBACK;
	else if ( state == COMMAND ) state = GENERAL;

	position = msg.position;
	speed = msg.speed;
	current = msg.current;

	well_defined = true;

	return SUCCESS;
}


CattyPartsError BrushlessMotor::set( const typename ServoFeedbackMsg::ConstPtr & msg ) override
{
	CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

	if ( state == NONE ) state = FEEDBACK;
	else if ( state == COMMAND ) state = GENERAL;

	position = msg->position;
	speed = msg->speed;
	current = msg->current;

	well_defined = true;

	return SUCCESS;
}
