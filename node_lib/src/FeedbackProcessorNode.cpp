#include <ros/ros.h>

using namespace Node;
            FeedbackProcessor fp;
            ros::Publisher processedFeedbackPublisher;
            ros::Subscriber teencyListner;
            ros::Publisher currentStatePublisher;
            std::string nodeName;
            std::string publishTopicName;
            std::string subscribeTopicName;
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

void teencyListnerCallback ( jcatty_teensy_msgs::InfoMsg::ConstPtr msg ){
	fp.processFeedback( msg );
	publish_processedFeedback( msg );
}

inline void publish_processedFeedback( jcatty_body_msgs::PartMsg ) const {
	processedFeedbackPublisher.publish( msg );
}

void publish_currentState() {
	body_msgs::PartMsg msg;
	fp.set_PartMsg( msg );
	currentStatePublisher.publish( msg );
}
