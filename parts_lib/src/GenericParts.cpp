#include <parts_lib/GenericParts.h>

template < typename Msg, typename CommandMsg, typename FeedbackMsg >
GenericParts<Msg, CommandMsg, FeedbackMsg>::GenericParts( std::string ChildName, PartID pID, Uint8 ID ) :
	child_name( ChildName ),
	part_id( pID ),
	id( ID ),
	valid( true ),
	well_defined( false ),
	state( NONE )
{}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const Msg & msg ) const 
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename Msg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const CommandMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename CommandMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const FeedbackMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename FeedbackMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const Msg & msg ) const 
{
	ROS_INFO("ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename Msg::ConstPtr & msg ) const
{
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const CommandMsg & msg ) const 
{
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename CommandMsg::ConstPtr & msg ) const
{
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const FeedbackMsg & msg ) const 
{
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename FeedbackMsg::ConstPtr & msg ) const 
{
	ROS_INFO("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const Msg & msg ) const
{
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename Msg::ConstPtr & msg ) const
{
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const CommandMsg & msg ) const
{
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and CommadnMsg=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename CommandMsg::ConstPtr & msg ) const
{
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const FeedbackMsg & msg ) const
{
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericParts<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename FeedbackMsg::ConstPtr & msg ) const {
	ROS_INFO("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set_msg( Msg & ) const {
    ROS_INFO("ERROR: No set_msg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
    return MESSAGE_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set_CommandMsg( CommandMsg & ) const {
	ROS_INFO("ERROR: No set_CommandMsg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
	return MESSAGE_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set_FeedbackMsg( FeedbackMsg & ) const {
	ROS_INFO("ERROR: No set_FeedbackMsg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
	return MESSAGE_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set( const CommandMsg & )
{
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set( const typename CommandMsg::ConstPtr & ) 
{
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set( const FeedbackMsg & )
{
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericParts<Msg, CommandMsg, FeedbackMsg>::set( const typename FeedbackMsg::ConstPtr & )
{
	ROS_INFO("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILUE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
PartID GenericParts<Msg, CommandMsg, FeedbackMsg>::get_part_id() const { return part_id; }


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
Uint8 GenericParts<Msg, CommandMsg, FeedbackMsg>::get_id() const { return id; }


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
bool GenericParts<Msg, CommandMsg, FeedbackMsg>::isValid() const { return valid; }
