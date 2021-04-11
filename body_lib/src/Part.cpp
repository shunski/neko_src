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
	brushedMotorSet   = std::vector<BrushedMotor>  ( pp.brushedMotorNum   );
	brushlessMotorSet = std::vector<BrushlessMotor>( pp.brushlessMotorNum );
	gyroSensorSet     = std::vector<GyroSensor>    ( pp.gyroSensorNum     );


	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(), int i=0;                            //for文エラー
		  it != kondoServoSet.end(); ++it, ++i ) *it = KondoServo(i);
	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(), int i=0;
		  it != brushedMotorSet.end(); ++it, ++i ) *it = BrushedMotor(i);
	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(), int i=0;
		  it != brushlessMotorSet.end(); ++it, ++i ) *it = BrushlessMotor(i);
	for ( std::vector<GyroSensor>::iterator it = gyroSensorSet.begin();
		  it != gyroSensorSet.end(); ++it, ++i ) *it = GyroSensor(i);
	return;
}

Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum ) :
    id( ID )
{
    for ( Uint8 i = 0; i<servoNum;          i++) kondoServoSet.push_back( KondoServo(i)); //servoSet -> kondoServoSet   // no matching function for call to ‘KondoServo::KondoServo(Uint8&)’
    for ( Uint8 i = 0; i<brushedMotorNum;   i++) brushedMotorSet.push_back( BrushedMotor(i));                           // ..
    for ( Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back( BrushlessMotor(i));                       // ..
    for ( Uint8 i = 0; i<gyroSensorNum;     i++) gyroSensorSet.push_back( GyroSensor(i));                               // ..
}

Part::Part ( PartID ID,                                                                                       //プロトタイプと違う型の引数？
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
    if ( kondoServoSet.size() != msg->kondoServoFeedbackSet.size() ) { //servoSet -> kondoServoSet          // msg->servoSet -> msg->kondoServoFeedbackSet
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [servoSet].");
        valid = false;
		return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorFeedbackSet.size() ) { // msg->brushedMotorSet -> msg->brushedMotorFeedbackSet
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [brushedMotorSet].");
        valid = false;
		return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorFeedbackSet.size() ) { //msg->brushlessMotorSet -> msg->brushlessMotorFeedbackSet
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [brushlessMotorSet].");
        valid = false;
		return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) { //teensy_msg/FeedbackMsg.msgにgyroSensorSetが定義されてない
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegInfoMsg] because of [gyroSensorSet].");
        valid = false;
		return;
    }

    for ( int i = 0; i < msg->kondoServoFeedbackSet.size(); i++ ) kondoServoSet[i].set( msg.servoSet[i] ); // .setの使い方あってる？
    for ( int i = 0; i < msg->brushedMotorFeedbackSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] ); //..
    for ( int i = 0; i < msg->brushlessMotorFeedbackSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] ); //..
    for ( int i = 0; i < msg->gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] ); // gyroSensorSetの定義がない
}

void Part::set ( teensy_msgs::CommandMsg & msg ) {
    if ( kondoServoSet.size() != msg->kondoServoCommandSet.size() ) { //
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [servoSet].");
        return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorCommandSet.size() ) { //
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushedMotorSet].");
        return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorCommandSet.size() ) { //
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [brushlessMotorSet].");
        return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) { // teensy_msgs/CommandMsg.msgにgyroSensorSetの定義がない
        ROS_INFO("ERROR: cannot set class Part from the message of type [teensy_msgs::ForelegCommandMsg] because of [gyroSensorSet].");
        return;
    }

    for ( int i = 0; i < msg.kondoServoCommandSet.size(); i++ ) kondoServoSet[i].set( msg.kondoServoSet[i] ); //
    for ( int i = 0; i < msg.brushedMotorCommandSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] ); //
    for ( int i = 0; i < msg.brushlessMotorCommandSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] ); //
    for ( int i = 0; i < msg.gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] ); //
}

void Part::set_CommandMsg( teensy_msgs::CommandMsg & msg ) {
	msg.kondoServoCommandSet.clear();
	msg.brushedMotorCommandSet.clear();
	msg.brushlessMotorCommandSet.clear();

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());                                                                        // teensy_msgs::CommandMsg has no member named "push_back" と //set_commandMsg(teensy_msgs::CommandMsg &)の引数がない
	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());                                                                        //..
	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it )
		msg.push_back( it->set_CommandMsg());                                                                        //..
}

body_msgs::PartMsg Part::get_PartMsg() const {
    body_msgs::PartMsg msg;

	for ( std::vector<KondoServo>::iterator it = kondoServoSet.begin(); it != kondoServoSet.end(); ++it )
		msg.push_back( it->set_msg());                                                                                //..
	for ( std::vector<BrushedMotor>::iterator it = brushedMotorSet.begin(); it != brushedMotorSet.end(); ++it )
		msg.push_back( it->set_msg());                                                                                //..
	for ( std::vector<BrushlessMotor>::iterator it = brushlessMotorSet.begin(); it != brushlessMotorSet.end(); ++it )
		msg.push_back( it->set_msg());                                                                                //..

    return msg;
}
