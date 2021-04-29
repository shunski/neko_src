// HeartrateSubscriberNode.cpp: implements HeartrateSubscriberNode class in Node.h
#include <node_lib/Node.h>
using namespace node;

HeartrateSubscriberNode::HeartrateSubscriberNode()
{
    heartrateListner = this->subscribe( heartrateTopicName,
			default_queue_size,
			&node::HeartrateSubscriberNode::heartrateSubscriberCallback, this);
    heartrate = ros::Duration( 1000 ); // super slow until initialized by the subscriber
}

void HeartrateSubscriberNode::heartrateSubscriberCallback( const support_msgs::HeartrateMsg::ConstPtr & msg )
{
    heartrate = msg->heartrate;
    heartPumped();
}
