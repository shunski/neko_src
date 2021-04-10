// Part.cpp implements Part class in Body.h
#include <body_lib/Body.h>
using namespace Body;

Part::Part( PartID ID ) : 
	id(ID), isValid(true)
{
	switch (PartID)
	{
	case( HEAD ):
	    for ( Uint8 i = 0; i<head_servo_num; i++ ) servoSet.push_back(KondoServo(i));
    	for ( Uint8 i = 0; i<head_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
    	for ( Uint8 i = 0; i<head_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
    	for ( Uint8 i = 0; i<head_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
		break;
    case( CHEST ):
        for ( Uint8 i = 0; i<chest_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<chest_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<chest_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<chest_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
    case( WAIST ):
        for ( Uint8 i = 0; i<waist_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<waist_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<waist_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<waist_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
    case( RFLEG ):
        for ( Uint8 i = 0; i<rf_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<rf_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<rf_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<rf_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
    case( LFLEG ):
        for ( Uint8 i = 0; i<lf_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<lf_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<lf_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<lf_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
    case( RHLEG ):
        for ( Uint8 i = 0; i<rh_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<rh_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<rh_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<rh_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
    case( LHLEG ):
        for ( Uint8 i = 0; i<lh_servo_num; i++ ) servoSet.push_back(KondoServo(i));
        for ( Uint8 i = 0; i<lh_brushedMotor_num; i++ ) brushedMotorSet.push_back(BrushedMotor(i));
        for ( Uint8 i = 0; i<lh_brushlessMotor_num; i++ ) brushlessMotorSet.push_back(BrushlessMotor(i));
        for ( Uint8 i = 0; i<lh_gyroSensorNum; i++ ) gyroSensorSet.push_back(GyroSensor(i));
        break;
	default:
		isValid = false;
		ROS_INFO("Invalid construction of Part by ID.")
		return;
}

Part::Part ( PartID ID, Uint8 servoNum, Uint8 brushedMotorNum, Uint8 brushlessMotorNum, Uint8 gyroSensorNum ) :
    id( ID )
{
    for (Uint8 i = 0; i<servoNum; i++) servoSet.push_back(KondoServo(i));
    for (Uint8 i = 0; i<brushedMotorNum; i++) brushedMotorSet.push_back(BrushedMotor(i));
    for (Uint8 i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back(BrushlessMotor(i));
    for (Uint8 i = 0; i<gyroSensorNum; i++) gyroSensorSet.push_back(GyroSensor(i));
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
    
}

body_msgs::PartMsg Part::get_PartMsg() const {
    jcatty_body_msgs::PartMsg msg;

    for ( int i = 0; i < servoSet.size(); i++ ) msg.servoSet.push_back( servoSet[i].get_KondoServoMsg() );
    for ( int i = 0; i < brushedMotorSet.size(); i++ ) msg.brushedMotorSet.push_back( brushedMotorSet[i].get_BrushedMotorMsg() );
    for ( int i = 0; i < brushlessMotorSet.size(); i++ ) msg.brushlessMotorSet.push_back( brushlessMotorMsg[i].get_BrushlessMotorMsg() );
    for ( int i = 0; i < gyroSensorSet.size(); i++ ) msg.gyroSensorSet[i].push_back( gyroSensorSet[i].get_gyroSensorMsg() );

    return msg;
}
