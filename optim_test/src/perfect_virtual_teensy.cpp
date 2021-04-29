// perfect_virtual_teensy.cpp: A made-up teensy model which does perfect work.
// The code here is written in such a way that its style to be as close to the
// actual teensy code as possible.

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <teensy_msgs/CommandMsg.h>
#include <teensy_msgs/FeedbackMsg.h>
#include <vector>
#include <limits.h>

// publisher and feedbackMsg are to be decalared with the global scope.
ros::Publisher pub;
teensy_msgs::FeedbackMsg feedbackMsg;
PartID PartId = RHLEG;
const PartProperties pp = get_properties_by_id( PartId );

// some useful object definitions thats represents the parts. See detailed definitions in parts_lib.
struct VirtualKondoServo
{
    // IDs
    Uint8 part_id;
    Uint8 id;

    // Command Information
    Uint8 current_limit;
    Uint8 temp_limit;
    Uint16 command_degree;
    Uint8 speed;
    Uint8 stretch;
    bool free;

    // Feedback Information
    Uint8 feedback_degree;
    Uint8 current;
    Uint8 temp;
    bool is_freed;
    void getWorkDone() {
        feedback_degree = command_degree;
        current = current_limit/2;
        temp = temp_limit/2;
        is_freed = free;
    }
};

struct VirtualBrushedMotor
{
    // IDs
    Uint8 part_id;
    Uint8 id;

    // Command Information
    Uint8 current_limit;
    Uint8 pwm;

    // Feedback Information
    Uint8 current;
    Uint16 rpm;
    void getWorkDone() {
        current = current_limit/2;
        rpm = rpm;
    }
};

struct VirtualBrushlessMotor
{
    // IDs
    Uint8 part_id;
    Uint8 id;

    // Command Information
    Int16 voltage;

    // Feedback Information
    Uint16 position;
    Int16 speed;
    Uint16 current;
    void getWorkDone(){
        position==1000? position==0 : position = position++;
        speed = 100;
        current = 100;
    }
};

struct VirtualMotionSensor
{
    // IDs
    Uint8 part_id;
    Uint8 id;

    // Feedback Information
    Uint16 accel_x;
    Uint16 accel_y;
    Uint16 accel_z;
    Uint16 gyro_x;
    Uint16 gyro_y;
    Uint16 gyro_z;
    Uint16 magnet_x;
    Uint16 magnet_y;
    Uint16 magnet_z;
};


// Declaring the objects
std::vector<VirtualKondoServo> kondoServoSet;
std::vector<VirtualBrushedMotor> brushedMotorSet;
std::vector<VirtualBrushlessMotor> brushlessMotorSet;
std::vector<VirtualMotionSensor> motionSensorSet;


// callback function for the CommandMsg. Implementation in below.
void teensy_callback( const teensy_msgs::CommandMsg::ConstPtr & receivedMsg );


// main function, a chunk of setup codes in it.
int main( int argc, char **argv ){
    // Initializing node
    ros::init( argc, argv, "perfect_teensy" );

    // Initializing the publisher and the subscriber
    ros::NodeHandle nh;
    pub = nh.advertise<teensy_msgs::FeedbackMsg>( "SomePartTeensyFeedback", 100 );
    ros::Subscriber sub = nh.subscribe<teensy_msgs::CommandMsg>( "SomePartTeensyCommand", 100, teensy_callback );

    // Initializing the virtual objects
    for ( Uint8 i=0; i<pp.kondoServoNum; i++ )
        kondoServoSet.push_back( VirtualKondoServo{ RHLEG, i, 0, 0, 0, 0, false, 0, 0, 0, 0, false } );
    for ( Uint8 i=0; i<pp.brushedMotorNum; i++)
        brushedMotorSet.push_back( VirtualBrushedMotor{ RHLEG, i, UCHAR_MAX, 0, 0, 0 } );
    for ( Uint8 i=0; i<pp.brushlessMotorNum; i++)
        brushlessMotorSet.push_back( VirtualBrushlessMotor{ RHLEG, i, 0, 0, 0, 0 } );
    for ( Uint8 i=0; i<pp.motionSensorNum; i++ )
        motionSensorSet.push_back( VirtualMotionSensor{ RHLEG, i, 0, 0, 0, 0, 0, 0, 0, 0, 0 } );

    // Initializing feedbackMsg
    feedbackMsg.part_id = PartId;
    for( int i=0; i<pp.kondoServoNum; i++ )
        feedbackMsg.kondoServoFeedbackSet.push_back( parts_msgs::KondoServoFeedbackMsg() );
    for( int i=0; i<pp.brushedMotorNum; i++ )
        feedbackMsg.brushedMotorFeedbackSet.push_back( parts_msgs::BrushedMotorFeedbackMsg() );
    for( int i=0; i<pp.brushlessMotorNum; i++ )
        feedbackMsg.brushlessMotorFeedbackSet.push_back( parts_msgs::BrushlessMotorFeedbackMsg() );
    for( int i=0; i<pp.motionSensorNum; i++ )
        feedbackMsg.motionSensorSet.push_back( parts_msgs::MotionSensorMsg() );

    // enters infinite loop, pumping callbacks. terminates when rosmaster say so.
    ros::spin();


    return 0;
}



// Callback Implementation
void teensy_callback( const teensy_msgs::CommandMsg::ConstPtr & receivedMsg )
{
    ros::Time timeOfCallbackStart = ros::Time::now();

    if( PartId != receivedMsg->part_id )
    {
        ROS_ERROR("-in teensy- part_id not match (PartId=%d while receivedMsg->part_id=%d)", PartId, receivedMsg->part_id );
        return;
    }

    // Reflecting the CommandMsg. The code here might be a little awkward but it is how things needs to be done for MC code.
    for( Uint8 i=0; i<pp.kondoServoNum; i++ )
    {
        if ( i != receivedMsg->kondoServoCommandSet[i].id )
        {
            ROS_ERROR("-in teensy- id not match(i=%d while receivedMsg->kondoServoCommandSet[i].id=%d)", i, receivedMsg->kondoServoCommandSet[i].id);
            return;
        }
        kondoServoSet[i].current_limit = receivedMsg->kondoServoCommandSet[i].current_limit;
        kondoServoSet[i].command_degree = receivedMsg->kondoServoCommandSet[i].command_degree;
        kondoServoSet[i].speed = receivedMsg->kondoServoCommandSet[i].speed;
        kondoServoSet[i].stretch = receivedMsg->kondoServoCommandSet[i].stretch;
        kondoServoSet[i].free = receivedMsg->kondoServoCommandSet[i].free;
    }

    for ( Uint8 i=0; i<pp.brushedMotorNum; i++ )
    {
        if ( i != receivedMsg->brushedMotorCommandSet[i].id )
        {
            ROS_ERROR("-in teensy- id not match");
            return;
        }
        brushedMotorSet[i].current_limit = receivedMsg->brushedMotorCommandSet[i].current_limit;
        brushedMotorSet[i].pwm = receivedMsg->brushedMotorCommandSet[i].pwm;
    }

    for ( Uint8 i=0; i<pp.brushlessMotorNum; i++ )
    {
        if ( i != receivedMsg->brushlessMotorCommandSet[i].id )
        {
            ROS_ERROR("-in teensy- id not match");
            return;
        }
        brushlessMotorSet[i].voltage = receivedMsg->brushlessMotorCommandSet[i].voltage;
    }

    // teensy does perfect work here
    for ( Uint8 i=0; i<pp.kondoServoNum; i++ ) kondoServoSet[i].getWorkDone();
    for ( Uint8 i=0; i<pp.brushedMotorNum; i++ ) brushedMotorSet[i].getWorkDone();
    for ( Uint8 i=0; i<pp.brushlessMotorNum; i++ ) brushlessMotorSet[i].getWorkDone();

    ros::Duration durationFromStart = ros::Time::now() - timeOfCallbackStart;
    ros::Duration sleepDuration;
    while ( durationFromStart < receivedMsg->expectedSceneDuration ) {
        sleepDuration = ( receivedMsg->expectedSceneDuration - durationFromStart ) * 0.5;
        sleepDuration.sleep();
        durationFromStart = ros::Time::now() - timeOfCallbackStart;
    }


    // Committing the results to the FeedbackMsg
    feedbackMsg.part_id = PartId;
    feedbackMsg.scene_id = receivedMsg->scene_id;
    feedbackMsg.actualCurrentSceneDuration = durationFromStart;

    for ( Uint8 i=0; i<pp.kondoServoNum; i++ ){
        feedbackMsg.kondoServoFeedbackSet[i].feedback_degree = kondoServoSet[i].feedback_degree;
        feedbackMsg.kondoServoFeedbackSet[i].current = kondoServoSet[i].current;
        feedbackMsg.kondoServoFeedbackSet[i].temp = kondoServoSet[i].temp;
        feedbackMsg.kondoServoFeedbackSet[i].is_freed = kondoServoSet[i].is_freed;
    }

    for ( Uint8 i=0; i<pp.brushedMotorNum; i++){
        feedbackMsg.brushedMotorFeedbackSet[i].current = brushedMotorSet[i].current;
        feedbackMsg.brushedMotorFeedbackSet[i].rpm = brushedMotorSet[i].rpm;
    }

    for ( Uint8 i=0; i<pp.brushlessMotorNum; i++){
        feedbackMsg.brushlessMotorFeedbackSet[i].position = brushlessMotorSet[i].position;
        feedbackMsg.brushlessMotorFeedbackSet[i].speed = brushlessMotorSet[i].speed;
        feedbackMsg.brushlessMotorFeedbackSet[i].current = brushlessMotorSet[i].current;
    }

    for ( Uint8 i=0; i<pp.motionSensorNum; i++ ){
        feedbackMsg.motionSensorSet[i].accel_x = motionSensorSet[i].accel_x;
        feedbackMsg.motionSensorSet[i].accel_y = motionSensorSet[i].accel_y;
        feedbackMsg.motionSensorSet[i].accel_z = motionSensorSet[i].accel_z;
        feedbackMsg.motionSensorSet[i].gyro_x = motionSensorSet[i].gyro_x;
        feedbackMsg.motionSensorSet[i].gyro_y = motionSensorSet[i].gyro_y;
        feedbackMsg.motionSensorSet[i].gyro_z = motionSensorSet[i].gyro_z;
        feedbackMsg.motionSensorSet[i].magnet_x = motionSensorSet[i].magnet_x;
        feedbackMsg.motionSensorSet[i].magnet_y = motionSensorSet[i].magnet_y;
        feedbackMsg.motionSensorSet[i].magnet_z = motionSensorSet[i].magnet_z;
    }

    pub.publish( feedbackMsg );
}
