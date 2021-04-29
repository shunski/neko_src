// HeartratePublisherNode.cpp: implements HeartratePublisherNode class in Node.h
#include <node_lib/Node.h>
using namespace node;

HeartratePublisherNode::HeartratePublisherNode():
    heartrate( ros::Duration( 0.5 )) // heartrate is 0.5 [Hz] by default
{
    heartratePublisher  = this->advertise<support_msgs::HeartrateMsg>( heartrateTopicName, default_queue_size );
}


CattyError HeartratePublisherNode::set_heartrate( const ros::Duration & heartrateIn ) {
    if ( heartrateIn < ros::Duration( 0.01 ))
    {
        ROS_INFO("Input heartrate too fast. It is instead set to 0.01");
        heartrate = ros::Duration( 0.01 );
        return WARNING;
    }
    else if ( heartrateIn > ros::Duration( 2.0 ) )
    {
        ROS_INFO("Input heartrate too slow. It is instead set to 2.0");
        heartrate = ros::Duration( 2.0 );
        return WARNING;
    }
    else
    {
        ROS_INFO("Heartrate newly set to ");
        heartrate = heartrateIn;
        return SUCCESS;
    }

    publish_newHeartrate();
    return SUCCESS;
}


CattyError HeartratePublisherNode::set_heartrate( const double heartrateIn ){
    return set_heartrate( ros::Duration( heartrateIn ));
}


void HeartratePublisherNode::publish_newHeartrate(){
    support_msgs::HeartrateMsg msg;
    msg.heartrate = heartrate;
    heartratePublisher.publish( msg );
}
