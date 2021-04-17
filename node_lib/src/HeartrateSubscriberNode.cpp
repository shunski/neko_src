// HeartrateSubscriberNode.cpp implements HeartrateSubscriberNode class in Node.h
#include <node_lib/Node.h>
using namespace Node;

HeartrateSubscriber::HeartrateSubscriber()
{
    HeartrateSubscriber this->subscribe<support_msgs::HeartrateMsg>(heartrateTopicName,
                                                                           queue_size,
                                                                           boost::bind(& Node::HeartrateSubscriber::HeartrateSubscriberCB, this));
    heartrate = ros::Duration(1000); // super slow until initialized by the subscriber
}

HeartrateSubscriber::HeartrateSubscriberCB(support_msgs::HeartrateMsg::ConstPtr msg)
{
    if (heartrate != msg.heartrate){
        heartrate = msg.heartrate;
        renewAllPublisherTimer();
    }
}
