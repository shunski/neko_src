// KondoServo.cpp: implements KondoServo.h
#include <parts_lib/KondoServo.h>

KondoServo::KondoServo ( PartID pID, Uint8 ID ) :
    GenericParts( "KondoServo", pID, ID )
{}


KondoServo::KondoServo( const KondoServo & original ):
	GenericParts( "KondoServo", PartID(original.part_id), original.id ),
	degree( original.degree ),
	temp_limit( original.temp_limit ),
	current_limit( original.current_limit ),
	speed( original.speed ),
    stretch( original.stretch )
{
	state = original.state;
	valid = original.valid;
	well_defined = original.well_defined;
}


KondoServo::KondoServo ( const ServoMsg &  msg ):
    GenericParts( "KondoServo", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


KondoServo::KondoServo ( const typename ServoMsg::ConstPtr &  msg ):
    GenericParts( "KondoServo", PartID(msg->part_id), msg->id )
{ this->set( msg ); }


KondoServo::KondoServo ( const ServoCommandMsg &  msg ):
    GenericParts( "KondoServo", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


KondoServo::KondoServo ( const typename ServoCommandMsg::ConstPtr &  msg ):
    GenericParts( "KondoServo", PartID(msg->part_id), msg->id )
{ this->set( msg ); }


KondoServo::KondoServo ( const ServoFeedbackMsg &  msg ):
    GenericParts( "KondoServo", PartID(msg.part_id), msg.id )
{ this->set( msg ); }


KondoServo::KondoServo ( const typename ServoFeedbackMsg::ConstPtr &  msg ):
    GenericParts( "KondoServo", PartID(msg->part_id), msg->id )
{ this->set( msg ); }



Uint16 KondoServo::get_degree () const { return degree; }
double KondoServo::get_degree_by_degree () const { return double(degree) * ( (135.0*2)/65535.0 - 135.0); }
Uint8 KondoServo::get_temp () const { return temp; }
Uint8 KondoServo::get_speed () const { return speed; }
Uint8 KondoServo::get_current() const { return current; }
Uint8 KondoServo::get_strech() const { return stretch;  }
Uint8 KondoServo::get_temp_limit() const { return temp_limit; }
Uint8 KondoServo::get_current_limit() const { return current_limit; }

void KondoServo::set_degree(Uint16 Degree) { degree = Degree; }
void KondoServo::set_temp(Uint8 Temp) { temp = Temp; }
void KondoServo::set_speed(Uint8 Speed) { speed = Speed;}
void KondoServo::set_current(Uint8 Current) { current = Current; }
void KondoServo::set_strech(Uint8 Stretch) { stretch = Stretch; }
void KondoServo::set_temp_limit(Uint8 Temp_limit) { temp_limit = Temp_limit; }
void KondoServo::set_current_limit(Uint8 Current_limit) { current_limit = Current_limit; }

void KondoServo::print() const
{
    if( valid ){
        ROS_INFO("Printing Information of <%s>: part_id = [%d], id = [%d], \ndegree\t= [%d], \ntemp_limit \t= [%d], \ncurrent_limit \t= [%d], \nspeed \t= [%d], \nstretch \t= [%d], \ntemp \t= [%d], \ncurrent \t= [%d] \n", child_name.c_str(), part_id, id, degree, temp_limit, current_limit, speed, stretch, temp, current );
    }
    else {
        ROS_INFO("This <%s> is not valid. Please exit the program.", child_name.c_str());
    }
}


CattyError KondoServo::set_msg( ServoMsg & msg ) const {
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
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.degree = degree;
	msg.temp_limit = temp_limit;
    msg.current_limit = current_limit;
	msg.speed = speed;
    msg.stretch = stretch;

	msg.temp = temp;
    msg.current = current;

    return SUCCESS;
}


CattyError KondoServo::set_CommandMsg( ServoCommandMsg & msg ) const {
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
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.degree = degree;
	msg.temp_limit = temp_limit;
    msg.current_limit = current_limit;
	msg.speed = speed;
    msg.stretch = stretch;

    return SUCCESS;
}


CattyError KondoServo::set_FeedbackMsg( ServoFeedbackMsg & msg ) const {
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
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is not suitable for setting the message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	msg.temp = temp;
    msg.current = current;

    return SUCCESS;
}


CattyError KondoServo::set( const ServoMsg & msg )
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

    degree = msg.degree;
	temp_limit = msg.temp_limit;
    current_limit = msg.current_limit;
	speed = msg.speed;
    stretch = msg.stretch;

	temp = msg.temp;
    current = msg.current;

	well_defined = true;

    return SUCCESS;
}


CattyError KondoServo::set( const typename ServoMsg::ConstPtr & msg )
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

    degree = msg->degree;
	temp_limit = msg->temp_limit;
    current_limit = msg->current_limit;
	speed = msg->speed;
    stretch = msg->stretch;

	temp = msg->temp;
    current = msg->current;

	well_defined = true;

    return SUCCESS;
}


CattyError KondoServo::set( const ServoCommandMsg & msg )
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

    degree = msg.degree;
	temp_limit = msg.temp_limit;
    current_limit = msg.current_limit;
	speed = msg.speed;
    stretch = msg.stretch;

	well_defined = true;

    return SUCCESS;
}


CattyError KondoServo::set( const ServoCommandMsg::ConstPtr & msg )
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

    degree = msg->degree;
	temp_limit = msg->temp_limit;
    current_limit = msg->current_limit;
	speed = msg->speed;
    stretch = msg->stretch;

	well_defined = true;

    return SUCCESS;
}


CattyError KondoServo::set( const ServoFeedbackMsg & msg )
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

	temp = msg.temp;
    current = msg.current;

	well_defined = true;

    return SUCCESS;
}


CattyError KondoServo::set( const typename ServoFeedbackMsg::ConstPtr & msg )
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

	temp = msg->temp;
    current = msg->current;

	well_defined = true;

    return SUCCESS;
}
