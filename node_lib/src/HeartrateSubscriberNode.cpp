// HeartrateSubscriberNode.cpp implements HeartrateSubscriberNode class in Node.h
#include <node_lib/Node.h>
using namespace Node;

HeartrateSubscriberNode::HeartrateSubscriberNode()
{
    heartrateListner = this->subscribe( heartrateTopicName,
			default_queue_size,
			&Node::HeartrateSubscriberNode::heartrateSubscriberCallback, this);
    heartrate = ros::Duration( 1000 ); // super slow until initialized by the subscriber
}

void HeartrateSubscriberNode::heartrateSubscriberCallback( const support_msgs::HeartrateMsg::ConstPtr & msg )
{
    if ( heartrate != msg->heartrate ){
        heartrate = msg->heartrate;
        renewAllPublisherTimer();
    }
}
