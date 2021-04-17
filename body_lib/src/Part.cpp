// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace Body;

Part::Part():
	part_id( NONE ), valid( true ), well_defined( false )
{}

Part::Part( PartID pID ) :
	part_id( pID ), valid( true ), well_defined( false )
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


Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 motionSensorNum ) :
    part_id( ID ), valid( false ), well_defined( false )
{
    for ( Uint8 i = 0; i<servoNum;          i++) kondoServoSet.push_back( KondoServo( part_id, i ));
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor( part_id, i ));
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor( part_id, i ));
    for ( Uint8 i = 0; i<motionSensorNum;   i++) motionSensorSet.push_back( MotionSensor( part_id, i ));
}


Part::Part ( PartID ID,
			const std::vector<KondoServo> & KondoServoSet,
			const std::vector<BrushedMotor> & BrushedMotorSet,
			const std::vector<BrushlessMotor> & BrushlessMotorSet ):
	part_id( ID ), valid( true ), well_defined( true ), state( COMMAND )
{
	PartProperties pp = get_properties_by_id( part_id );
	if ( pp.kondoServoNum     != KondoServoSet.size()   ||
		 pp.brushedMotorNum   != BrushedMotorSet.size() ||
		 pp.brushlessMotorNum != BrushlessMotorSet.size() )
	{
		ROS_INFO("ERROR: cannot construct Part instance from parts sets because sizes do not match. ");
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


Part::Part( PartID pID , const std::vector<parts_msgs::KondoServoCommandMsg> & KondoServoCommandSet,
                   const std::vector<parts_msgs::BrushedMotorCommandMsg> & BrushedMotorCommandSet,
                   const std::vector<parts_msgs::BrushlessMotorCommandMsg> & BrushlessMotorCommandSet ):
	part_id( pID ), valid( true ), well_defined( true ), state( COMMAND )
{
	PartProperties pp = get_properties_by_id( part_id );
	if ( pp.kondoServoNum     != KondoServoCommandSet.size()   ||
         pp.brushedMotorNum   != BrushedMotorCommandSet.size() ||
         pp.brushlessMotorNum != BrushlessMotorCommandSet.size() )
	{
		ROS_INFO("ERROR: cannot construct Part instance from parts sets because sizes do not match. ");
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


CattyError Part::set ( const teensy_msgs::FeedbackMsg::ConstPtr & msg ) {
	if ( part_id != msg->part_id ) {
		ROS_INFO("OBJECT_CONSTRUCTION_FAILUE: Could not set the Part object by FeedbackMsg since part_id does not match.");
		valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
	}

	if ( state==INVALID ) state == FEEDBACK;
	else if ( state==COMMAND ) state == GENERAL;

	for ( std::vector<parts_msgs::KondoServoFeedbackMsg>::const_iterator it=msg->kondoServoFeedbackSet.begin(); it!=msg->kondoServoFeedbackSet.end(); ++it )
		kondoServoSet.push_back(*it);
	for ( std::vector<parts_msgs::BrushedMotorFeedbackMsg>::const_iterator it=msg->brushedMotorFeedbackSet.begin(); it!=msg->brushedMotorFeedbackSet.end(); ++it )
		brushedMotorSet.push_back(*it);
	for ( std::vector<parts_msgs::BrushlessMotorFeedbackMsg>::const_iterator it=msg->brushlessMotorFeedbackSet.begin(); it!=msg->brushlessMotorFeedbackSet.end(); ++it )
		brushlessMotorSet.push_back(*it);
	for ( std::vector<parts_msgs::MotionSensorMsg>::const_iterator it=msg->motionSensorSet.begin(); it!=msg->motionSensorSet.end(); ++it )
		motionSensorSet.push_back(*it);

    well_defined = true;

	return SUCCESS;
}


CattyError Part::set ( const typename teensy_msgs::CommandMsg::ConstPtr & msg ) {
	if ( part_id != msg->part_id ) {
		ROS_INFO("OBJECT_CONSTRUCTION_FAILUE: Could not set the Part object by FeedbackMsg since part_id does not match.");
		valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
	}

	if ( state==INVALID ) state == COMMAND;
	else if ( state==FEEDBACK ) state == GENERAL;

	for ( std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator it=msg->kondoServoCommandSet.begin(); it!=msg->kondoServoCommandSet.end(); ++it )
		kondoServoSet.push_back(*it);
	for ( std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator it=msg->brushedMotorCommandSet.begin(); it!=msg->brushedMotorCommandSet.end(); ++it )
		brushedMotorSet.push_back(*it);
	for ( std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator it=msg->brushlessMotorCommandSet.begin(); it!=msg->brushlessMotorCommandSet.end(); ++it )
		brushlessMotorSet.push_back(*it);

	well_defined = true;

	return SUCCESS;
}


CattyError Part::set_CommandMsg( teensy_msgs::CommandMsg & msg ) {
	if ( !valid ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUER: Invalid Part Object is trying to set the PartMsg. Please exit the program.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if ( !( state == GENERAL || state == COMMAND )){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE:This Part object is not suitable for setting the CommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if ( !well_defined ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This Part Object is ill-defined and thus could not set the teensy_msgs::CommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperties pp = get_properties_by_id( part_id );
		msg.kondoServoCommandSet.reserve( pp.kondoServoNum );
		msg.brushedMotorCommandSet.reserve( pp.brushedMotorNum );
		msg.brushlessMotorCommandSet.reserve( pp.brushlessMotorNum );

	} else if ( part_id != msg.part_id ) {

		ROS_INFO("PART_ID_NOT_MATCH: Could not set the PartMsg since PartID does not match.");
		valid = false;
		return ID_NOT_MATCH;

	} else {

		msg.kondoServoCommandSet.clear();
		msg.brushedMotorCommandSet.clear();
		msg.brushlessMotorCommandSet.clear();

    }

	msg.kondoServoCommandSet.clear();
	msg.brushedMotorCommandSet.clear();
	msg.brushlessMotorCommandSet.clear();

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
}


CattyError Part::set_PartMsg( body_msgs::PartMsg & msg ){
	if ( !valid ) {
		ROS_INFO("ERROR: Invalid Part Object is trying to set the PartMsg. Please exit the program.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if ( state != GENERAL ){
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This Part object is not suitable for setting the CommandMsg.");
		return MESSAGE_CONSTRUCTION_FAILUE;
    }

	if ( !well_defined ) {
		ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This Part Object is ill-defined and thus could not set the PartMsg.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperties pp = get_properties_by_id( part_id );
		msg.kondoServoSet.reserve( pp.kondoServoNum );
		msg.brushedMotorSet.reserve( pp.brushedMotorNum );
		msg.brushlessMotorSet.reserve( pp.brushlessMotorNum );
		msg.motionSensorSet.reserve( pp.motionSensorNum );

	} else if ( part_id != msg.part_id ) {

		ROS_INFO("PART_ID_NOT_MATCH: Could not set the PartMsg since PartID does not match.");
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
	
	return SUCCESS;

}


bool Part::isValid() const { return valid; }


bool Part::isWellDefined() const { return well_defined; }
