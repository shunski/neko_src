// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace Body;

Part::Part(){
	valid = false;
}

Part::Part( PartID ID ) :
	id( ID ), valid( true )
{
	PartProperties pp = get_properties_by_id( id );
	kondoServoSet     = std::vector<KondoServo>    ( pp.kondoServoNum     );
	brushedMotorSet   = std::vector<brushedMotor>  ( pp.brushedMotorNum   );
	brushlessMotorNum = std::vector<brushlessMotor>( pp.brushlessMotorNum );
	gyroSensorNum     = std::vector<GyroSensor>    ( pp.gyroSensorNum     );


	for ( std::vector<KondoServo>::iterator     it, int i=0; it = kondoServoSet.begin();     ++it, ++i ) it = KondoServo(i);
	for ( std::vector<BrushedMotor>::iterator   it, int i=0; it = brushedMotorSet.begin();   ++it, ++i ) it = BrushedMotor(i);
	for ( std::vector<BrushlessMotor>::iterator it, int i=0; it = brushlessMotorSet.begin(); ++it, ++i ) it = BrushlessMotor(i);
	for ( std::vector<GyroSensor>::iterator     it, int i=0; it = gyroSensorSet.begin();     ++it, ++i ) it = GyroSensor(i);
	return;
}

Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum ) :
    id( ID )
{
    for ( Uint8 i = 0; i<servoNum;          i++) servoSet.push_back( KondoServo(i));
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor(i));
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor(i));
    for ( Uint8 i = 0; i<gyroSensorNum;     i++) gyroSensorSet.push_back( GyroSensor(i));
}

Part::Part ( PartID ID,
			std::vector<KondoServo> KondoServoSet,
			std::vector<BrushedMotor> BrushedMotorSet,
			std::vector<BrushlessMotor> BrushlessMotorSet ):
	id( ID )
{
	PartProperties pp = get_properties_by_id( id );
	if ( pp.kondoServoNum != KondoServoSet.size()     ||
		 pp.brushedMotorNum != BrushedMotorSet.size() ||
		 pp.brushedMotorNum != BrushlessMotorSet.size() )
	{
		valid = false;
		return;
	}
	kondoServoSet     = KondoServoSet;
	brushedMotorSet   = BrushedMotorSet;
	brushlessMotorSet = BrushlessMotorSet;
}

void Part::set ( teensy_msgs::FeedbackMsg::ConstPtr & msg ) {
    if ( servoSet.size() != msg->servoSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [servoSet].");
        isValid = false;
		return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [brushedMotorSet].");
        isValid = false;
		return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [brushlessMotorSet].");
        isValid = false;
		return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [gyroSensorSet].");
        isValid = false;
		return;
    }

    for ( int i = 0; i < msg->servoSet.size(); i++ ) servoSet[i].set( msg.servoSet[i] );
    for ( int i = 0; i < msg->brushedMotorSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] );
    for ( int i = 0; i < msg->brushlessMotorSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] );
    for ( int i = 0; i < msg->gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] );
}

void Part::set ( teensy_msgs::CommandMsg & msg ) {
    if ( servoSet.size() != msg->servoSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [servoSet].");
        return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushedMotorSet].");
        return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushlessMotorSet].");
        return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [gyroSensorSet].");
        return;
    }

    for ( int i = 0; i < msg->servoSet.size(); i++ ) servoSet[i].set( msg.servoSet[i] );
    for ( int i = 0; i < msg->brushedMotorSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] );
    for ( int i = 0; i < msg->brushlessMotorSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] );
    for ( int i = 0; i < msg->gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] );
}

void Part::set_CommandMsg( teensy_msgs::CommandMsg & msg ) {
	msg.kondoServoCommandSet.clear();
	msg.brushedMotorCommandSet.clear();
	msg.brushlessMotorCommandSet.clear();

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());
	for ( std::vector<brushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedmotorSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());
	for ( std::vector<brushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessmotorSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());
}

body_msgs::PartMsg Part::get_PartMsg() const {
    body_msgs::PartMsg msg;

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it )
		msg.push_back( it->set_msg());
	for ( std::vector<brushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedmotorSet.end(); ++it )
		msg.push_back( it->set_msg());
	for ( std::vector<brushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessmotorSet.end(); ++it )
		msg.push_back( it->set_msg());

    return msg;
}
