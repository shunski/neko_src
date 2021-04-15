#include <parts_msgs/GenericParts.h>

GenericParts::GenericParts( std::string ChildName, PartID pID, Uint8 ID, ObjectState State) :
	child_name( ChildName ),
	part_id( pID ),
	id( ID ),
	valid( true ),
	well_defined( false ),
	state( NONE )
{}

CattyPartsError GenericParts::check_msg_id( Msg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError GenericParts::check_msg_id( typename Msg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError GenericParts::check_msg_id( CommandMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError GenericParts::check_msg_id( typename CommandMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError GenericParts::check_msg_id( FeedbackMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


CattyPartsError GenericParts::check_msg_id( typename FeedbackMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}

void GenericParts::print_msg_part_id_error( Msg & msg ){
	ROS_INFO("ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, messageName.c_str(), msg.part_id );
}
void GenericParts::print_msg_part_id_error( typename Msg::ConstPtr & msg ){
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->part_id );
}
void GenericParts::print_msg_part_id_error( CommandMsg & msg ){
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg.part_id );
}
void GenericParts::print_msg_part_id_error( typename CommandMsg::ConstPtr & msg ){
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->part_id );
}
void GenericParts::print_msg_part_id_error( FeedbackMsg & msg ){
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg.part_id );
}
void GenericParts::print_msg_part_id_error( typename FeedbackMsg::ConstPtr & msg ){
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg->part_id );
}

void GenericParts::print_msg_id_error( Msg & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg.id );
}
void GenericParts::print_msg_id_error( typename Msg::ConstPtr & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->id );
}
void GenericParts::print_msg_id_error( CommandMsg & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and CommadnMsg=%d", child_name.c_str(), part_id, msg.id );
}
void GenericParts::print_msg_id_error( typename CommandMsg::ConstPtr & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->id );
}
void GenericParts::print_msg_id_error( FeedbackMsg & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg.id );
}
void GenericParts::print_msg_id_error( typename FeedbackMsg::ConstPtr & msg ){
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg->id );
}


virtual CattyPartsError GenericParts::set( const CommandMsg & ){
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


virtual CattyPartsError GenericParts::set( const typename CommandMsg::ConstPtr & ){
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


virtual CattyPartsError GenericParts::set( const FeedbackMsg & ){
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}

virtual CattyPartsError GenericParts::set( const typename FeedbackMsg::ConstPtr & ){
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


PartID GenericParts::get_part_id() const { return part_id; }

Uint8 GenericParts::get_id() const { return id; }

bool GenericParts::isValid() const { return valid; }
