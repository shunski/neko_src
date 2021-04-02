// Part.cpp implements Part class in Body.h
#include <Body.h>
using namespace Body;


Part::Part ( std::string PartName, unsigned char servoNum, unsigned char brushedMotorNum, unsigned char brushlessMotorNum, unsigned char gyroSensorNum ) :
    partName(PartName)
{
    for (unsigned char i = 0; i<servoNum; i++) servoSet.push_back(KondoServo());
    for (unsigned char i = 0; i<brushedMotorNum; i++) brushedMotorSet.push_back(BrushedMotor());
    for (unsigned char i = 0; i<brushlessMotorNum; i++) brushlessMotorSet.push_back(BrushlessMotor());
    for (unsigned char i = 0; i<gyroSensorNum; i++) gyroSensorSet.push_back(GyroSensor());
}

void Part::set ( jcatty_teensy_msgs::InfoMsg::ConstPtr & msg ) {
    if ( servoSet.size() != msg->servoSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegInfoMsg] because of [servoSet].");
        return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegInfoMsg] because of [brushedMotorSet].");
        return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegInfoMsg] because of [brushlessMotorSet].");
        return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegInfoMsg] because of [gyroSensorSet].");
        return;
    }

    for ( int i = 0; i < msg->servoSet.size(); i++ ) servoSet[i].set( msg.servoSet[i] );
    for ( int i = 0; i < msg->brushedMotorSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] );
    for ( int i = 0; i < msg->brushlessMotorSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] );
    for ( int i = 0; i < msg->gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] );
}

void Part::set ( jcatty_teensy_msgs::CommandMsg msg ) {
    if ( servoSet.size() != msg->servoSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegCommandMsg] because of [servoSet].");
        return;
    }
    if ( brushedMotorSet.size() != msg->brushedMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegCommandMsg] because of [brushedMotorSet].");
        return;
    }
    if ( brushlessMotorSet.size() != msg->brushlessMotorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegCommandMsg] because of [brushlessMotorSet].");
        return;
    }
    if ( gyroSensorSet.size() != msg->gyroSensorSet.size() ) {
        ROS_INFO("ERROR: cannot set class Part from the message of type [jcatty_teensy_msgs::ForelegCommandMsg] because of [gyroSensorSet].");
        return;
    }

    for ( int i = 0; i < msg->servoSet.size(); i++ ) servoSet[i].set( msg.servoSet[i] );
    for ( int i = 0; i < msg->brushedMotorSet.size(); i++ ) brushedMotorSet[i].set( msg.brushedMotorSet[i] );
    for ( int i = 0; i < msg->brushlessMotorSet.size(); i++ ) brushlessMotorSet[i].set( msg.brushlessMotorSet[i] );
    for ( int i = 0; i < msg->gyroSensorSet.size(); i++ ) gyroSensorSet[i].set( msg.gyroSensorSet[i] );
}

void Part::set_CommandMsg( jcatty_teensy_msgs::CommandMsg msg ) {
    set_msgs();
}

jcatty_body_msgs::PartMsg Part::get_PartMsg() const {
    jcatty_body_msgs::PartMsg msg;

    for ( int i = 0; i < servoSet.size(); i++ ) msg.servoSet.push_back( servoSet[i].get_KondoServoMsg() );
    for ( int i = 0; i < brushedMotorSet.size(); i++ ) msg.brushedMotorSet.push_back( brushedMotorSet[i].get_BrushedMotorMsg() );
    for ( int i = 0; i < brushlessMotorSet.size(); i++ ) msg.brushlessMotorSet.push_back( brushlessMotorMsg[i].get_BrushlessMotorMsg() );
    for ( int i = 0; i < gyroSensorSet.size(); i++ ) msg.gyroSensorSet[i].push_back( gyroSensorSet[i].get_gyroSensorMsg() );

    return msg;
}
