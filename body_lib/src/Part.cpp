// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace Body;

Part::Part():
	part_id( NONE ),
	valid( false )
{}

Part::Part( PartID ID ) :
	part_id( ID ), valid( true )
{
	PartProperties pp = get_properties_by_id( part_id );
	kondoServoSet    .reserve( pp.kondoServoNum     );
	brushedMotorSet  .reserve( pp.brushedMotorNum   );
	brushlessMotorSet.reserve( pp.brushlessMotorNum );
	gyroSensorSet    .reserve( pp.gyroSensorNum     );


	for ( auto t = std::make_tuple( std::vector<KondoServo>::iterator(kondoServoSet.begin()), 0 );
		  std::get<std::vector<KondoServo>::iterator>(t) != kondoServoSet.end();
	      ++std::get<std::vector<KondoServo>::iterator>(t), ++std::get<int>(t) )
		 *std::get<std::vector<KondoServo>::iterator>(t) = KondoServo( std::get<int>(t), part_id );
    for ( auto t = std::make_tuple( std::vector<BrushedMotor>::iterator(brushedMotorSet.begin()), 0 );
          std::get<std::vector<BrushedMotor>::iterator>(t) != brushedMotorSet.end(); 
          ++std::get<std::vector<BrushedMotor>::iterator>(t), ++std::get<int>(t) )
         *std::get<std::vector<BrushedMotor>::iterator>(t) = BrushedMotor( std::get<int>(t), part_id );
	for ( auto t = std::make_tuple( std::vector<BrushlessMotor>::iterator(brushlessMotorSet.begin()), 0 );
          std::get<std::vector<BrushlessMotor>::iterator>(t) != brushlessMotorSet.end(); 
          ++std::get<std::vector<BrushlessMotor>::iterator>(t), ++std::get<int>(t) )
         *std::get<std::vector<BrushlessMotor>::iterator>(t) = BrushlessMotor( std::get<int>(t), part_id );
	for ( auto t = std::make_tuple( std::vector<GyroSensor>::iterator(gyroSensorSet.begin()), 0 );
          std::get<std::vector<GyroSensor>::iterator>(t) != gyroSensorSet.end(); 
          ++std::get<std::vector<GyroSensor>::iterator>(t), ++std::get<int>(t) )
         *std::get<std::vector<GyroSensor>::iterator>(t) = GyroSensor( std::get<int>(t), part_id );
	return;
}

Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum ) :
    part_id( ID )
{
    for ( Uint8 i = 0; i<servoNum;          i++) kondoServoSet.push_back( KondoServo( i, part_id ));
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor( i, part_id ));
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor( i, part_id ));
    for ( Uint8 i = 0; i<gyroSensorNum;     i++) gyroSensorSet.push_back( GyroSensor( i, part_id ));
}

Part::Part ( PartID ID,
			std::vector<KondoServo> KondoServoSet,
			std::vector<BrushedMotor> BrushedMotorSet,
			std::vector<BrushlessMotor> BrushlessMotorSet ):
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
	kondoServoSet     = KondoServoSet;
	brushedMotorSet   = BrushedMotorSet;
	brushlessMotorSet = BrushlessMotorSet;
}

void Part::set ( teensy_msgs::FeedbackMsg::ConstPtr & msg ) {
    if ( kondoServoSet.size() != msg->kondoServoFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [servoSet].");
        valid = false;
		return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [brushedMotorSet].");
        valid = false;
		return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorFeedbackSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::FeedbackMsg] because of [brushlessMotorSet].");
        valid = false;
		return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [gyroSensorSet].");
        valid = false;
		return;
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

    for ( auto t = std::make_tuple( std::vector<GyroSensor>::iterator( gyroSensorSet.begin()),
          std::vector<parts_msgs::GyroSensorMsg>::const_iterator( msg->gyroSensorSet.begin()));
          std::get<std::vector<GyroSensor>::iterator>(t) != gyroSensorSet.end();
          ++std::get<std::vector<GyroSensor>::iterator>(t),
          ++std::get<std::vector<parts_msgs::GyroSensorMsg>::const_iterator>(t) )
        std::get<std::vector<GyroSensor>::iterator>(t) -> set( *std::get<std::vector<parts_msgs::GyroSensorMsg>::const_iterator>(t) );

}

CattyError Part::set ( teensy_msgs::CommandMsg & msg ) {
    if ( kondoServoSet.size() != msg->kondoServoCommandSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [servoSet].");
		valid = false;
        return CONSTRUCT_ERROR;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorCommandSet.size() ) { 
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushedMotorSet].");
		valid = false;
        return CONSTRUCT_ERROR;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorCommandSet.size() ) { 
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushlessMotorSet].");
		valid = false;
        return CONSTRUCT_ERROR;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [gyroSensorSet].");
		valid = false;
        return CONSTRUCT_ERROR;
    }
	if ( msg.id != id ){
		ROS_INFO("ERROR: cannotr set Part instance from the message of type [teensy_msgs::CommandMsg] because id does not match.");
		valid = false;
		return CONSTRUCT_ERROR;
	}
    for ( std::vector<KondoServo>::iterator object_iterator = kondoServoSet.begin(),
          std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator msg_iterator  = msg->kondoServoCommandSet.begin();
          object_iterator != kondoServoSet.end();
          ++object_iterator, ++msg_iterator )
        object_iterator->set( *msg_iterator );
    for ( std::vector<BrushedMotor>::iterator object_iterator = brushedMotorSet.begin(),
          std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator msg_iterator  = msg->brushedMotorCommandSet.begin();
          object_iterator != brushedMotorSet.end();
          ++object_iterator, ++msg_iterator )
        object_iterator->set( *msg_iterator );
    for ( std::vector<BrushlessMotor>::iterator object_iterator = brushlessMotorSet.begin(),
          std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator msg_iterator  = msg->brushlessMotorCommandSet.begin();
          object_iterator != brushlessMotorSet.end();
          ++object_iterator, ++msg_iterator )
        object_iterator->set( *msg_iterator );

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
	if ( part_id != msg->part_id ) {
		return ID_NOT_MATCH;
	}
	
}

bool isValid(){
	return valid;
}
