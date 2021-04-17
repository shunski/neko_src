#include <ros/ros.h>
#include <node_lib/Node.h>

using namespace Node;


FeedbackProcessorNode(PartID ID, std::string NodeName, std::string PublishTopicName, std::string SubscribeTopicName, std::string HeartrateFeedbackName)
	fp(ID),
	nodeName( NodeName ),
	publishTopicName( PublishTopicName ),
	subscribeTopicName( SubscribeTopicName ),
	heartrateFeedbackName( HeartrateFeedbackName )
{
	processedFeedbackPublisher = this->advertise<body_msgs::PartMsg>(publishTopicName, queue_size );
	currentStatePublisher = this->advertise<body_msgs::PartMsg>( heartrateFeedbackName, queue_size );
	teensyListner = this->advertise<teensy_msgs::InfoMs>(subscribeTopicName, queue_size,
                                                                boost::bind( &Node::FeedbackProcessorNode::teensyListerCallback, this ));
}

void FeedbackProcessorNode::teencyListnerCallback ( teensy_msgs::FeedbackMsg::ConstPtr msg ){
	fp.processFeedback( msg );
	publish_processedFeedback( msg );
}

inline void FeedbackProcessorNode::publish_processedFeedback( body_msgs::PartMsg ) const {
	processedFeedbackPublisher.publish( msg );
}

void FeedbackProcessorNode::publish_currentState() {
	body_msgs::PartMsg msg;
	fp.set_PartMsg( msg );
	currentStatePublisher.publish( msg );
}
