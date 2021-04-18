#include <ros/ros.h>
#include <node_lib/Node.h>

using namespace Node;


FeedbackProcessorNode(PartID ID, std::string PartName )
	fp( ID ),
	nodeName( PartName+"FeedbackProcessor" ),
	toMotionControllerFeedbackTopicName( PartName+"ProcessedFeedback" ),
	toMotionControllerFinishActionTopicName( PartName+"FinishAction" ),
	fromMotionControllerTeensyCommandTopicName( PartName+"TeensyCommand" ),
	fromTeensySubscribeTopicName( PartName+"TeensyFeedback" ),
	fromMotionControllerActionStartNotifierTopicName( PartName+"ActionState" ),
	heartrateFeedbackName( PartName+"HeartrateFeedback" ),
	valid( true )
{
	processedFeedbackPublisher = this->advertise<body_msgs::PartMsg>( toMotionControllerPublishTopicName, queue_size );
	actionEndReporter = this->advertise<support_msgs::ActionEndReporterMsg>( toMotionControllerFinishActionTopicName, queue_size );
	teensyCommandSubscriber = this->subscribe<teensy_msgs::FeedbackMsg>( fromMotionControllerTeensyCommandTopicName,
	                                                                boost::bind(& Node::FeedbackProcessorNode::commandsListnerCallback, this ));
	currentStatePublisher = this->advertise<std_msgs::bool>( heartrateFeedbackName, queue_size );
	teensyListner = this->advertise<teensy_msgs::FeedbackMsg>( FromTeensySubscribeTopicName, queue_size,
                                                                boost::bind(& Node::FeedbackProcessorNode::teensyListerCallback, this ));
	actionStartListner = this->advertiseService<support_msgs::actionStartNotifierMsg>( FromTeensySubscribeTopicName,
                                                boost::bind( &Node::FeedbackProcessorNode::actionStartListnerCallback, this ));
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& Node::FeedbackProcessorNode::publish_currentState(), this ));
	fpValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind(& Node::FeedbackProcessorNode::checkFpValidness(), this ))
}


void FeedbackProcessorNode::teensyCommandSubscriber ( teensy_msgs::CommandMsg::ConstPtr msg ){
	bool isActionFinished = pf.add_pendingScenes( msg );
	if ( isActionFinished ){
		support_msgs::ActionEndReporterMsg endReporter;
		fp.set_endReporterMsg( endReporter );
		actionEndReporter
	}
}


void FeedbackProcessorNode::teensyListnerCallback ( teensy_msgs::FeedbackMsg::ConstPtr msg ){
	Part processedFeedback = fp.processFeedback( msg );
	PartMsg publishingMsg;
	processFeedback.set_PartMsg( publishingMsg );
	processedFeedbackPublisher.publish( publishMsg );
}


void FeedbackProcessorNode::publish_currentState() {
	std_msgs::bool msg = valid;
	currentStatePublisher.publish( msg );
}


bool FeedbackProcessorNode::actionStartListnerCallback( support_msgs::ActionStartNotifierMsg & msg ) {
	fp.start_action( msg, nodeName );
}


void FeedbackProcessorNode::checkFpValidness() {
	valid = fp.isValid();
}
