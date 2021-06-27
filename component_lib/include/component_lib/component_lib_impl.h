template < typename Msg, typename CommandMsg, typename FeedbackMsg >
GenericActuator<Msg, CommandMsg, FeedbackMsg>::GenericActuator( std::string ChildName, PartID pID, Uint8 ID ) :
	child_name( ChildName ),
	part_id( pID ),
	id( ID ),
	valid( true ),
	well_defined( false ),
	state( INVALID )
{}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const Msg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename Msg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const CommandMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename CommandMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const FeedbackMsg & msg ) const
{
	if ( part_id != msg.part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg.id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::check_msg_id( const typename FeedbackMsg::ConstPtr & msg ) const
{
	if ( part_id != msg->part_id ) return PART_ID_NOT_MATCH;
	if ( id != msg->id ) return ID_NOT_MATCH;
	return SUCCESS;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const Msg & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename Msg::ConstPtr & msg ) const
{
	ROS_ERROR("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const CommandMsg & msg ) const
{
	ROS_ERROR("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename CommandMsg::ConstPtr & msg ) const
{
	ROS_ERROR("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const FeedbackMsg & msg ) const
{
	ROS_ERROR("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg.part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_part_id_error( const typename FeedbackMsg::ConstPtr & msg ) const
{
	ROS_ERROR("PART_ID_NOT_MATCH: PartID did not match: %s object=%d and FeedbackMsg=%d", child_name.c_str(), part_id, msg->part_id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const Msg & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename Msg::ConstPtr & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and Msg=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const CommandMsg & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and CommadnMsg=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename CommandMsg::ConstPtr & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and CommandMsg=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const FeedbackMsg & msg ) const
{
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg.id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
void GenericActuator<Msg, CommandMsg, FeedbackMsg>::print_msg_id_error( const typename FeedbackMsg::ConstPtr & msg ) const {
	ROS_ERROR("ID_NOT_MATCH: ID did not match: %s object=%d and Feedback=%d", child_name.c_str(), part_id, msg->id );
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set_msg( Msg & ) const {
    ROS_ERROR("ERROR: No set_msg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
    return MESSAGE_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set_CommandMsg( CommandMsg & ) const {
	ROS_ERROR("ERROR: No set_CommandMsg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
	return MESSAGE_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set_FeedbackMsg( FeedbackMsg & ) const {
	ROS_ERROR("ERROR: No set_FeedbackMsg() method is not defined in the derived class <%s>. Please exit the program.", child_name.c_str());
	return MESSAGE_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set( const CommandMsg & )
{
	ROS_ERROR("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set( const typename CommandMsg::ConstPtr & )
{
	ROS_ERROR("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set( const FeedbackMsg & )
{
	ROS_ERROR("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
CattyError GenericActuator<Msg, CommandMsg, FeedbackMsg>::set( const typename FeedbackMsg::ConstPtr & )
{
	ROS_ERROR("ERROR: No set() method is defined in the derived class <%s> for the mesaage of this type. Please exit the program.", child_name.c_str());
	valid = false;
	return OBJECT_CONSTRUCTION_FAILURE;
}


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
PartID GenericActuator<Msg, CommandMsg, FeedbackMsg>::get_part_id() const { return part_id; }


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
Uint8 GenericActuator<Msg, CommandMsg, FeedbackMsg>::get_id() const { return id; }


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
ObjectState GenericActuator<Msg, CommandMsg, FeedbackMsg>::get_state() const { return state; }


template < typename Msg, typename CommandMsg, typename FeedbackMsg >
bool GenericActuator<Msg, CommandMsg, FeedbackMsg>::isValid() const { return valid; }
