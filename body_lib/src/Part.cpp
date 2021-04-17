// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace Body;

Part::Part():
	part_id( NONE ), valid( false )
{}

Part::Part( PartID pID ) :
	part_id( pID ), valid( true )
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
    part_id( ID )
{
    for ( Uint8 i = 0; i<servoNum;          i++) kondoServoSet.push_back( KondoServo( part_id, i ));
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor( part_id, i ));
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor( part_id, i ));
    for ( Uint8 i = 0; i<motionSensorNum;   i++) motionSensorSet.push_back( MotionSensor( part_id, i ));
}

Part::Part ( PartID ID,
			const std::vector<KondoServo> KondoServoSet,
			const std::vector<BrushedMotor> BrushedMotorSet,
			const std::vector<BrushlessMotor> BrushlessMotorSet ):
	part_id( ID )
{
	PartProperties pp = get_properties_by_id( part_id );
	if ( pp.kondoServoNum     != KondoServoSet.size()   ||
		 pp.brushedMotorNum   != BrushedMotorSet.size() ||
		 pp.brushlessMotorNum != BrushlessMotorSet.size() )
	{
		ROS_INFO("ERROR: cannot construct Part instance from parts sets because sizes do not match. ");
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

CattyError Part::set ( const teensy_msgs::FeedbackMsg::ConstPtr & msg ) {
    if ( kondoServoSet.size() != msg->kondoServoFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [servoSet].");
        valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [brushedMotorSet].");
        valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [brushlessMotorSet].");
        valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( motionSensorSet.size() != msg->motionSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [motionSensorSet].");
        valid = false;
		return OBJECT_CONSTRUCTION_FAILUE;
    }

    for ( auto t = std::make_tuple( std::vector<KondoServo>::iterator( kondoServoSet.begin()),
	      std::vector<parts_msgs::KondoServoFeedbackMsg>::const_iterator( msg->kondoServoFeedbackSet.begin()));
	      std::get<std::vector<KondoServo>::iterator>(t) != kondoServoSet.end();
	      ++std::get<std::vector<KondoServo>::iterator>(t),
	      ++std::get<std::vector<parts_msgs::KondoServoFeedbackMsg>::const_iterator>(t) )
		std::get<std::vector<KondoServo>::iterator>(t) -> set( *std::get<std::vector<parts_msgs::KondoServoFeedbackMsg>::const_iterator>(t) );

    for ( auto t = std::make_tuple( std::vector<BrushedMotor>::iterator( brushedMotorSet.begin()),
          std::vector<parts_msgs::BrushedMotorFeedbackMsg>::const_iterator( msg->brushedMotorFeedbackSet.begin()));
          std::get<std::vector<BrushedMotor>::iterator>(t) != brushedMotorSet.end();
          ++std::get<std::vector<BrushedMotor>::iterator>(t),
          ++std::get<std::vector<parts_msgs::BrushedMotorFeedbackMsg>::const_iterator>(t) )
        std::get<std::vector<BrushedMotor>::iterator>(t) -> set( *std::get<std::vector<parts_msgs::BrushedMotorFeedbackMsg>::const_iterator>(t) );

    for ( auto t = std::make_tuple( std::vector<BrushlessMotor>::iterator( brushlessMotorSet.begin()),
          std::vector<parts_msgs::BrushlessMotorFeedbackMsg>::const_iterator( msg->brushlessMotorFeedbackSet.begin()));
          std::get<std::vector<BrushlessMotor>::iterator>(t) != brushlessMotorSet.end();
          ++std::get<std::vector<BrushlessMotor>::iterator>(t),
          ++std::get<std::vector<parts_msgs::BrushlessMotorFeedbackMsg>::const_iterator>(t) )
        std::get<std::vector<BrushlessMotor>::iterator>(t) -> set( *std::get<std::vector<parts_msgs::BrushlessMotorFeedbackMsg>::const_iterator>(t) );

    for ( auto t = std::make_tuple( std::vector<MotionSensor>::iterator( motionSensorSet.begin()),
          std::vector<parts_msgs::MotionSensorMsg>::const_iterator( msg->motionSensorSet.begin()));
          std::get<std::vector<MotionSensor>::iterator>(t) != motionSensorSet.end();
          ++std::get<std::vector<MotionSensor>::iterator>(t),
          ++std::get<std::vector<parts_msgs::MotionSensorMsg>::const_iterator>(t) )
        std::get<std::vector<MotionSensor>::iterator>(t) -> set( *std::get<std::vector<parts_msgs::MotionSensorMsg>::const_iterator>(t) );

	return SUCCESS;
}

CattyError Part::set ( const typename teensy_msgs::CommandMsg::ConstPtr & msg ) {

    if ( kondoServoSet.size() != msg.kondoServoCommandSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::CommandMsg] because of [servoSet].");
		valid = false;
        return  OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( brushedMotorSet.size() != msg.brushedMotorCommandSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::CommandMsg] because of [brushedMotorSet].");
		valid = false;
        return  OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( brushlessMotorSet.size() != msg.brushlessMotorCommandSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::CommandMsg] because of [brushlessMotorSet].");
		valid = false;
        return  OBJECT_CONSTRUCTION_FAILUE;
    }
    if ( motionSensorSet.size() != msg.motionSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::CommandMsg] because of [motionSensorSet].");
		valid = false;
        return OBJECT_CONSTRUCTION_FAILUE;
    }
	if ( msg.part_id != part_id ){
		ROS_INFO("ERROR: cannot set Part instance from the message of type [teensy_msgs::CommandMsg] because PartID does not match.");
		valid = false;
		return PART_ID_NOT_MATCH;
	}

	if ( state==INVALID ) state == COMMAND;
	else if ( state==FEEDBACK ) state == GENERAL;

	for ( std::vector<KondoServo>::const_iterator it=msg.kondoServoSet.begin(); it!=msg.kondoServoSet.end(); ++it )
		kondoServoSet.push_back(*it);
	for ( std::vector<BrushedMotor>::const_iterator it=msg.brushedMotorSet.begin(); it!=msg.brushedMotorSet.end(); ++it )
		brushedMotorSet.push_back(*it);
	for ( std::vector<BrushlessMotor>::const_iterator it=msg.brushlessMotorSet.begin(); it!=msg.brushlessMotorSet.end(); ++it )
		brushlessMotorSet.push_back(*it);

	return SUCCESS;
}

void Part::set_CommandMsg( teensy_msgs::CommandMsg & msg ) {
	msg.kondoServoCommandSet.clear();
	msg.brushedMotorCommandSet.clear();
	msg.brushlessMotorCommandSet.clear();

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it )
		msg.kondoServoCommandSet.push_back( it->set_CommandMsg());
	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it )
		msg.brushedMotorCommandSet.push_back( it->set_CommandMsg());
	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it )
		msg.brushlessMotorCommandSet.push_back( it->set_CommandMsg());
}

body_msgs::PartMsg Part::get_PartMsg() const {
    body_msgs::PartMsg msg;
	this->set_CommandMsg( msg );
    return msg;
}

CattyError set_PartMsg( body_msgs::PartMsg & msg ){
	if ( !valid ) {
		ROS_INFO("ERROR: Invalid Part Object is trying to set the PartMsg. Please exitt the program.");
		return MESSAGE_CONSTRUCTION_FAILUE;
	}

	if( msg.part_id == 0 ){

		msg.part_id = part_id;

		PartProperty pp = get_part_property( part_id );
		msg.kondoServoSet.reserve( pp.kondoServoNum );
		msg.brushedMotorSett.reserve( pp.brushedMotorNum );
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

	for ( std::vector<KondoServo>::const_iterator it = kondoServoSet.begin(); it!=kondoServoSet.end(); ++it )
		msg.kondoServoSet.push_back(*it);
	for ( std::vector<BrushedMotor>::const_iterator it = brushedMotorSet.begin(); it!=brushedMotorSet.end(); ++it )
		msg.brushedMotorSet.push_back(*it);
	for ( std::vector<BrushlessMotor>::const_iterator it = brushlessMotorSet.begin(); it!=brushlessMotorSet.end(); ++it )
		msg.brushlessMotorSet.push_back(*it);
	for ( std::vector<MotionSensor>::const_iterator it = motionSensorSet.begin(); it!=motionSensorSet.end(); ++it )
		msg.motionSensorSet.push_back(*it);

	return SUCCESS;

}

bool Part::isValid(){ return valid; }
