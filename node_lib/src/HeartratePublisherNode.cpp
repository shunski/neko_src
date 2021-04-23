// HeartratePublisherNode.cpp: implements HeartratePublisherNode class in Node.h
#include <node_lib/Node.h>
using namespace Node;

HeartratePublisherNode::HeartratePublisherNode():
    heartrate( ros::Duration( 0.5 )) // heartrate is 0.5 [Hz] by default
{
    heartratePublisher  = this->advertise( heartrateTopicName, default_queue_size );
    nodeRegistrationServer = this->ServiceServer( nodeRegistrationTopicName, Node::HeartratePublisherNode::register_node );
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
        ROS_INFO("Input heartrate too slow. It is instead set to 2");
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
    return;
}

void HeartratePublisherNode::publish_newHeartrate(){
    heartratePublisher.publish();
}


CattyError HeartratePublisherNode::set_heartrate( const double heartrateIn ){
    return set_heartrate( ros::Duration( heartrateIn ));
}

void currentStateSubscriberCallback( const support_msgs::HeartrateMsg::ConstPtr & msg ){
    if ( msg.node_name )
    nodeDisable ros::Timer = this->createTimer( ros::Duration, , true );
}
