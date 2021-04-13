#include <parts_msgs/Parts.h>

Parts::Parts( PartID pID, Uint8 ID, ObjectState State) : 
	part_id( pID ), 
	id( ID ), 
	valid( true ), 
	well_defined( false ) 
{}


CattyPartsError Parts::heck_msg_id( Msg & msg ) const :
{
	if ( part_id == msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id == msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError Parts::heck_msg_id( typename Msg::ConstPtr & msg ) const {
	if ( part_id == msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id == msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError Parts::heck_msg_id( CommandMsg & msg ) const {
	if ( part_id == msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id == msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError Parts::heck_msg_id( typename CommandMsg::ConstPtr & msg ) const :{
	if ( part_id == msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id == msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError Parts::heck_msg_id( typename FeedbackMsg::ConstPtr & msg ) const {
	if ( part_id == msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id == msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}

void Parts::print_id_error( Uint8 pID, Uint8 ID) const {
	ROS_INFO("%s: Object(part_id:%d, id:%d) <-> Message(part_id:%d, id:%d)", 
		get_error_description( error ).c_str(), 
		part_id, id, 
		pID, ID);
}

CattyPartsError Parts::set( Msg & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) {
		print_id_error( msg.part_id, msg.id );
		return error;
	}
	if ( state == COMMAND || state == FEEDBACK ) state = OBJECT;

	command_data.set( msg );
	feedback_data.set( msg );

	well_defined = true;
	return SUCCESS;
}

 
CattyPartsError Parts::set( typename Msg::ConstPtr & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) {
		print_id_error( msg->part_id, msg->id );
		return error;
	}

	if ( state == NONE || state == COMMAND || state == FEEDBACK ) state = OBJECT;

	command_data.set( msg );
	feedback_data.set( msg );
	well_defined = true;
	return SUCCESS;
}


CattyPartsError Parts::set( CommandMsg & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) {
		print_id_error( msg.part_id, msg.id );
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	if ( state == Feedback ) state = OBJECT;

	command_data.set( msg );
	well_defined = true;
	return SUCCESS;
}


CattyPartsError Parts::set( typename CommandMsg::ConstPtr & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) {
		print_id_error( msg->part_id, msg->id );
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	if ( state == Feedback ) state = OBJECT;

	command_data.set( msg );
	well_defined = true;
	return SUCCESS;
}


CattyPartsError Parts::set( FeedbackMsg & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ){
		print_id_error( msg.part_id, msg.id );
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	if ( state == Command ) state = OBJECT;

	command_data.set( msg );
	well_defined = true;
	return SUCCESS;
}


CattyPartsError Parts::set( typename FeedbackMsg::ConstPtr & msg ){
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ){
		print_id_error( msg->part_id, msg->id );
		return error;
	}

	if ( state == NONE ) state = COMMAND;
	if ( state == Command ) state = OBJECT;

	command_data.set( msg );
	well_defined = true;
	return SUCCESS;
}


CattyPartsError Parts::set_msg( Msg & msg ) const {
	CattyPartError error = check_id( msg );
    if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) return error;
	if ( state != OBJECT ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is not suitable for setting the message.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is ill-defined. Could not set a message.");
        return MESSAGE_CONSTRUCTION_FAILUE;
	}

	command_data.set( msg );
	feedback_data.set( msg );

	return SUCCESS;
}


CattyParsError Parts::set_CommandMsg( CommadMsg & ) const {
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) return error;

	if ( !( state == COMMAND || state == OBJECT )) {
        ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is not suitable for setting the command message.");
        return MESSAGE_CONSTRUCTION_FAILUE;
    }
	if ( !valid || !well_defined ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is ill-defined. Could not set a command message.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	command_data.set( msg );

	return SUCCESS;
}


CattyPartsError Parts::set_FeedbackMsg( FeedbackMsg & ) const {
	CattyPartError error = check_id( msg );
	if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) return error;

	if ( !( state == Feedback || state == OBJECT )){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is not suitable for setting the feedback message.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}
	if ( !valid || !well_defined ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This object is ill-defined. Could not set a feedback message.");
	}

	feedback_data.set( msg );

	return SUCCESS;
}


