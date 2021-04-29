// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace body;

Part::Part():
	part_id( NONE ), scene_id( USHRT_MAX ), valid( true ), well_defined( false ), state( INVALID )
{}



Part::Part( PartID pID ) :
	part_id( pID ), scene_id( USHRT_MAX ), valid( true ), well_defined( false ), state( INVALID )
{
	PartProperties pp = get_properties_by_id( part_id );

	kondoServoSet    .reserve( pp.kondoServoNum     );
	brushedMotorSet  .reserve( pp.brushedMotorNum   );
	brushlessMotorSet.reserve( pp.brushlessMotorNum );
	motionSensorSet  .reserve( pp.motionSensorNum   );

	for ( int i=0; i<pp.kondoServoNum; i++ )
		kondoServoSet.push_back( KondoServo( part_id, i ));
	for ( int i=0; i<pp.brushedMotorNum; i++ )
		brushedMotorSet.push_back( BrushedMotor( part_id, i ));
	for ( int i=0; i<pp.brushlessMotorNum; i++ )
		brushlessMotorSet.push_back( BrushlessMotor( part_id, i ));
	for ( int i=0; i<pp.motionSensorNum; i++ )
		motionSensorSet.push_back( MotionSensor( part_id, i ));

	return;
}


Part::Part( PartID pID, Uint16 SceneID ) :
	part_id( pID ), scene_id( SceneID ), valid( true ), well_defined( false ), state( INVALID )
{
	PartProperties pp = get_properties_by_id( part_id );

	kondoServoSet    .reserve( pp.kondoServoNum     );
	brushedMotorSet  .reserve( pp.brushedMotorNum   );
	brushlessMotorSet.reserve( pp.brushlessMotorNum );
	motionSensorSet  .reserve( pp.motionSensorNum   );

	for ( int i=0; i<pp.kondoServoNum; i++ )
		kondoServoSet.push_back( KondoServo( part_id, i ));
	for ( int i=0; i<pp.brushedMotorNum; i++ )
		brushedMotorSet.push_back( BrushedMotor( part_id, i ));
	for ( int i=0; i<pp.brushlessMotorNum; i++ )
		brushlessMotorSet.push_back( BrushlessMotor( part_id, i ));
	for ( int i=0; i<pp.motionSensorNum; i++ )
		motionSensorSet.push_back( MotionSensor( part_id, i ));

	return;
}


Part::Part( const teensy_msgs::CommandMsg::ConstPtr & msg ) :
	part_id( PartID(msg->part_id)), scene_id( msg->scene_id ), valid( true ), well_defined( true ), state( INVALID )
{
	CattyError error = set( msg );
	if ( error!=SUCCESS ) valid = false;
	return;
}


Part::Part( const teensy_msgs::FeedbackMsg::ConstPtr & msg ) :
	part_id( PartID(msg->part_id)), scene_id( msg->scene_id ), valid( true ), well_defined( true ), state( INVALID )
{
	CattyError error = set( msg );
	if ( error!=SUCCESS ) valid = false;
	return;
}


Part::Part( const body_msgs::PartMsg::ConstPtr & msg ):
	part_id( PartID(msg->part_id)), scene_id( msg->scene_id ), valid( true ), well_defined( true ), state( INVALID )
{
	CattyError error = set( msg );
	if ( error!=SUCCESS ) valid = false;
	return;
}


Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum ) :
    part_id( ID ), scene_id( USHRT_MAX ), valid( false ), well_defined( false ), state( INVALID )
{
    for ( Uint8 i = 0; i<servoNum;          i++) kondoServoSet.push_back( KondoServo( part_id, i ));
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor( part_id, i ));
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor( part_id, i ));
    for ( Uint8 i = 0; i<motionSensorNum;   i++) motionSensorSet.push_back( MotionSensor( part_id, i ));
}


Part::Part ( PartID pID,
			const std::vector<KondoServo> & KondoServoSet,
			const std::vector<BrushedMotor> & BrushedMotorSet,
			const std::vector<BrushlessMotor> & BrushlessMotorSet ):
	part_id( pID ), scene_id( USHRT_MAX ), valid( true ), well_defined( true ), state( COMMAND )
{
	PartProperties pp = get_properties_by_id( part_id );
	if ( pp.kondoServoNum     != KondoServoSet.size()   ||
		 pp.brushedMotorNum   != BrushedMotorSet.size() ||
		 pp.brushlessMotorNum != BrushlessMotorSet.size() )
	{
		ROS_ERROR("Could not construct Part instance from parts sets because sizes do not match. ");
		well_defined = false;
		state = INVALID;
		valid = false;
		return;
	}
	kondoServoSet.reserve( pp.kondoServoNum );
	brushedMotorSet.reserve( pp.brushedMotorNum );
	brushlessMotorSet.reserve( pp.brushlessMotorNum );
	for ( std::vector<KondoServo>::const_iterator it=KondoServoSet.begin(); it!=KondoServoSet.end(); it++ )
		kondoServoSet.push_back(*it);
	for ( std::vector<BrushedMotor>::const_iterator it=BrushedMotorSet.begin(); it!=BrushedMotorSet.end(); it++ )
		brushedMotorSet.push_back(*it);
	for ( std::vector<BrushlessMotor>::const_iterator it=BrushlessMotorSet.begin(); it!=BrushlessMotorSet.end(); it++ )
		brushlessMotorSet.push_back(*it);

	motionSensorSet.reserve( pp.motionSensorNum );
	for ( Uint8 i=0; i<pp.motionSensorNum; i++ )
		motionSensorSet.push_back( MotionSensor( part_id, i ) );
}


Part::Part( Uint16 sceneId, PartID pID ,
			const std::vector<parts_msgs::KondoServoCommandMsg> & KondoServoCommandSet,
        	const std::vector<parts_msgs::BrushedMotorCommandMsg> & BrushedMotorCommandSet,
            const std::vector<parts_msgs::BrushlessMotorCommandMsg> & BrushlessMotorCommandSet ):
	part_id( pID ), scene_id( sceneId ), valid( true ), well_defined( true ), state( COMMAND )
{
	PartProperties pp = get_properties_by_id( part_id );
	if ( pp.kondoServoNum     != KondoServoCommandSet.size()   ||
         pp.brushedMotorNum   != BrushedMotorCommandSet.size() ||
         pp.brushlessMotorNum != BrushlessMotorCommandSet.size() )
	{
		ROS_ERROR("Could not construct Part instance from the set of CommandMsg of parts because sizes do not match. ");
		well_defined = false;
		state = INVALID;
		valid = false;
		return;
	}
	kondoServoSet.reserve( pp.kondoServoNum );
	brushedMotorSet.reserve( pp.brushedMotorNum );
	brushlessMotorSet.reserve( pp.brushlessMotorNum );

	for ( std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator it=KondoServoCommandSet.begin(); it!=KondoServoCommandSet.end(); it++ )
		kondoServoSet.push_back( KondoServo(*it) );
	for ( std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator it=BrushedMotorCommandSet.begin(); it!=BrushedMotorCommandSet.end(); it++ )
		brushedMotorSet.push_back( BrushedMotor(*it) );
	for ( std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator it=BrushlessMotorCommandSet.begin(); it!=BrushlessMotorCommandSet.end(); it++ )
		brushlessMotorSet.push_back( BrushlessMotor(*it) );


	motionSensorSet.reserve( pp.motionSensorNum );
	for ( Uint8 i=0; i<pp.motionSensorNum; i++ )
		motionSensorSet.push_back( MotionSensor( part_id, i ) );

}


Part::Part( const Part & original ):
	part_id(original.part_id),
	scene_id(original.scene_id),
	kondoServoSet(original.kondoServoSet),
	brushedMotorSet(original.brushedMotorSet),
	brushlessMotorSet(original.brushlessMotorSet),
	motionSensorSet(original.motionSensorSet),
	valid(original.valid),
	well_defined(original.well_defined),
	state(original.state)
{}


void Part::operator=( const Part & original )
{
	if( part_id != original.part_id )
		ROS_INFO("Invalid use of assignment operator for two Parts. Operation failed.");
	scene_id = original.scene_id;
	kondoServoSet = original.kondoServoSet;
	brushedMotorSet = original.brushedMotorSet;
	brushlessMotorSet = original.brushlessMotorSet;
	motionSensorSet = original.motionSensorSet;
	valid = original.valid;
	well_defined = original.well_defined;
	state = original.state;
}


PartID Part::get_part_id() const {
	return part_id;
}

Uint16 Part::get_scene_id() const {
	return scene_id;
}

ObjectState Part::get_state() const {
	return state;
}


CattyError Part::set ( const teensy_msgs::FeedbackMsg::ConstPtr & msg ) {
	if ( part_id != msg->part_id ) {
		ROS_ERROR("OBJECT_CONSTRUCTION_FAILURE: Could not set the Part object by FeedbackMsg since part_id does not match.");
		valid = false;
		return OBJECT_CONSTRUCTION_FAILURE;
	}

	if ( state==INVALID ) state = FEEDBACK;
	else if ( state==COMMAND ) state = GENERAL;

	for ( int i=0; i<kondoServoSet.size(); ++i )
		kondoServoSet[i].set( msg->kondoServoFeedbackSet[i] );
	for ( int i=0; i<brushedMotorSet.size(); ++i )
		brushedMotorSet[i].set( msg->brushedMotorFeedbackSet[i] );
	for ( int i=0; i<brushlessMotorSet.size(); ++i )
		brushlessMotorSet[i].set( msg->brushlessMotorFeedbackSet[i] );
	for ( int i=0; i<motionSensorSet.size(); ++i )
		motionSensorSet[i].set( msg->motionSensorSet[i] );

    well_defined = true;

	return SUCCESS;
}


CattyError Part::set ( const typename teensy_msgs::CommandMsg::ConstPtr & msg ) {
	if ( part_id != msg->part_id ) {
		ROS_ERROR("OBJECT_CONSTRUCTION_FAILURE: Could not set the Part object by FeedbackMsg since part_id does not match.");
		valid = false;
		return OBJECT_CONSTRUCTION_FAILURE;
	}

	if ( state==INVALID ) state = COMMAND;
	else if ( state==FEEDBACK ) state = GENERAL;

	for ( int i=0; i<kondoServoSet.size(); ++i )
		kondoServoSet[i].set( msg->kondoServoCommandSet[i] );
	for ( int i=0; i<brushedMotorSet.size(); ++i )
		brushedMotorSet[i].set( msg->brushedMotorCommandSet[i] );
	for ( int i=0; i<brushlessMotorSet.size(); ++i )
		brushlessMotorSet[i].set( msg->brushlessMotorCommandSet[i] );

	well_defined = true;

	return SUCCESS;
}


CattyError Part::set ( const typename body_msgs::PartMsg::ConstPtr & msg ) {
    if ( part_id != msg->part_id ) {
        ROS_ERROR("OBJECT_CONSTRUCTION_FAILURE: Could not set the Part object by body_msgs::PartMsg since part_id does not match.");
        valid = false;
        return OBJECT_CONSTRUCTION_FAILURE;
    }

	state = GENERAL;

	for ( int i=0; i<kondoServoSet.size(); ++i )
		kondoServoSet[i].set( msg->kondoServoSet[i] );
	for ( int i=0; i<brushedMotorSet.size(); ++i )
		brushedMotorSet[i].set( msg->brushedMotorSet[i] );
	for ( int i=0; i<brushlessMotorSet.size(); ++i )
		brushlessMotorSet[i].set( msg->brushlessMotorSet[i] );
	for ( int i=0; i<motionSensorSet.size(); ++i )
		motionSensorSet[i].set( msg->motionSensorSet[i] );

	well_defined = true;

	return SUCCESS;
}


CattyError Part::set_CommandMsg( teensy_msgs::CommandMsg & msg ) {
	if ( !valid ) {
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: Invalid Part Object is trying to set the PartMsg. Please exit the program.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if ( !( state == GENERAL || state == COMMAND )){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part object is not suitable for setting the CommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if ( !well_defined ) {
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part Object is ill-defined and thus could not set the teensy_msgs::CommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperties pp = get_properties_by_id( part_id );
		msg.kondoServoCommandSet.reserve( pp.kondoServoNum );
		msg.brushedMotorCommandSet.reserve( pp.brushedMotorNum );
		msg.brushlessMotorCommandSet.reserve( pp.brushlessMotorNum );

	} else if ( part_id != msg.part_id ) {

		ROS_ERROR("PART_ID_NOT_MATCH: Could not set the PartMsg since PartID does not match.");
		valid = false;
		return ID_NOT_MATCH;

	} else {

		msg.kondoServoCommandSet.clear();
		msg.brushedMotorCommandSet.clear();
		msg.brushlessMotorCommandSet.clear();

    }

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it ){
		parts_msgs::KondoServoCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.kondoServoCommandSet.push_back( commandMsg );
	}

	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it ){
		parts_msgs::BrushedMotorCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.brushedMotorCommandSet.push_back( commandMsg );
	}

	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it ){
		parts_msgs::BrushlessMotorCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.brushlessMotorCommandSet.push_back( commandMsg );
	}

	msg.scene_id = scene_id;

	return SUCCESS;
}


CattyError Part::set_PartCommandMsg( body_msgs::PartCommandMsg & msg ){
	if ( !valid ) {
		ROS_ERROR("MESSAGE CONSTRUCTION FAILUE: Invalid Part Object is trying to set the PartMsg. Please exit the program.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if ( state != GENERAL && state != COMMAND ){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part object (state=[%s]) is not suitable for setting the PartCommandMsg.", get_description(state).c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if ( !well_defined ) {
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part Object is ill-defined and thus could not set the PartCommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperties pp = get_properties_by_id( part_id );
		msg.kondoServoCommandSet.reserve( pp.kondoServoNum );
		msg.brushedMotorCommandSet.reserve( pp.brushedMotorNum );
		msg.brushlessMotorCommandSet.reserve( pp.brushlessMotorNum );

	} else if ( part_id != msg.part_id ) {

		ROS_ERROR("PART_ID_NOT_MATCH: Could not set the PartCommandMsg since PartID does not match.");
		valid = false;
		return ID_NOT_MATCH;

	} else {

		msg.kondoServoCommandSet.clear();
		msg.brushedMotorCommandSet.clear();
		msg.brushlessMotorCommandSet.clear();

	}

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it ){
		parts_msgs::KondoServoCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.kondoServoCommandSet.push_back( commandMsg );
	}

	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it ){
		parts_msgs::BrushedMotorCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.brushedMotorCommandSet.push_back( commandMsg );
	}

	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it ){
		parts_msgs::BrushlessMotorCommandMsg commandMsg;
		it->set_CommandMsg( commandMsg );
		msg.brushlessMotorCommandSet.push_back( commandMsg );
	}

	return SUCCESS;

}


CattyError Part::set_PartMsg( body_msgs::PartMsg & msg ){
	if ( !valid ) {
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: Invalid Part Object is trying to set the PartMsg. Please exit the program.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if ( state != GENERAL ){
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part object (state=[%s]) is not suitable for setting the PartMsg.", get_description(state).c_str());
		return MESSAGE_CONSTRUCTION_FAILURE;
    }

	if ( !well_defined ) {
		ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This Part Object is ill-defined and thus could not set the PartMsg.");
		return MESSAGE_CONSTRUCTION_FAILURE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperties pp = get_properties_by_id( part_id );
		msg.kondoServoSet.reserve( pp.kondoServoNum );
		msg.brushedMotorSet.reserve( pp.brushedMotorNum );
		msg.brushlessMotorSet.reserve( pp.brushlessMotorNum );
		msg.motionSensorSet.reserve( pp.motionSensorNum );

	} else if ( part_id != msg.part_id ) {

		ROS_ERROR("PART_ID_NOT_MATCH: Could not set the PartMsg since PartID does not match.");
		valid = false;
		return ID_NOT_MATCH;

	} else {

		msg.kondoServoSet.clear();
		msg.brushedMotorSet.clear();
		msg.brushlessMotorSet.clear();
		msg.motionSensorSet.clear();

	}

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it ){
		parts_msgs::KondoServoMsg generalMsg;
		it->set_msg( generalMsg );
		msg.kondoServoSet.push_back( generalMsg );
	}

	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it ){
		parts_msgs::BrushedMotorMsg generalMsg;
		it->set_msg( generalMsg );
		msg.brushedMotorSet.push_back( generalMsg );
	}

	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it ){
		parts_msgs::BrushlessMotorMsg generalMsg;
		it->set_msg( generalMsg );
		msg.brushlessMotorSet.push_back( generalMsg );
	}

	msg.scene_id = scene_id;

	return SUCCESS;

}


bool Part::isValid() const { return valid; }

bool Part::isWellDefined() const { return well_defined; }

void Part::set_RandomPartCommandMsg( body_msgs::PartCommandMsg & msg ){
	msg.kondoServoCommandSet.clear();
	msg.brushedMotorCommandSet.clear();
	msg.brushlessMotorCommandSet.clear();

	PartProperties pp = get_properties_by_id( part_id );

	for ( Uint8 i=0; i<pp.kondoServoNum; ++i ){
		parts_msgs::KondoServoCommandMsg commandMsg;
		kondoServoSet[i].set_RandomCommandMsg( commandMsg );
		msg.kondoServoCommandSet.push_back( commandMsg );
	}

	for ( Uint8 i=0; i<pp.brushedMotorNum; ++i ){
		parts_msgs::BrushedMotorCommandMsg commandMsg;
		brushedMotorSet[i].set_RandomCommandMsg( commandMsg );
		msg.brushedMotorCommandSet.push_back( commandMsg );
	}

	for ( Uint8 i=0; i<pp.brushlessMotorNum; ++i ){
		parts_msgs::BrushlessMotorCommandMsg commandMsg;
		brushlessMotorSet[i].set_RandomCommandMsg( commandMsg );
		msg.brushlessMotorCommandSet.push_back( commandMsg );
	}
}
