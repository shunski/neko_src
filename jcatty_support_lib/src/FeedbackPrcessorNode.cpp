// FeedbackProcessorNode.cpp implements FeedbackProcessorNode class in Node.h
#include <jcatty_support_lib/Node.h>
using namespace Node;

FeedbackProcessorNode::FeedbackProcessorNode(std::string NodeName, std::string PublishTopicName, std::string SubscribeTopicName):
    nodeName( NodeName ),
    publishTopicName( PublishTopicName ),
    subscribeTopicName( SubscribeTopicName )
{
    legProcessedFeedbackPublisher = this->advertise<jcatty_body_msgs::PartMsg>(publishTopicName, queue_size);
    teencyListner = this->subscribe<jcatty_teensy_msgs::InfoMsg>(subscribeTopicName,
                                                                  queue_size,
                                                                  boost::bind(& Node::FeedbackProcessorNode::teencyListnerCallback, this ));
}

void FeedbackProcessorNode::teencyListnerCallback ( jcatty_teensy_msgs::InfoMsg::ConstPtr receivedMsg ){
    Part processedInfo = fc.processFeedback( receivedMsg );
    jcatty_body_msgs::Part partMsg = processedInfo.get_PartMsg();
    publish_processedFeedbask( partMsg );
}

inline void FeedbackProcessorNode::publish_processedFeedback( jcatty_body_msgs::PartMsg msg ) const {
    ROS_INFO("Publishing [%s] from [%s]", publishTopicName.c_str(), nodeName.c_str());
    legProcessedFeedbackPublisher.publish( msg );
}
