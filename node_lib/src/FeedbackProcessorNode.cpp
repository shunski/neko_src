#include <node_lib/Node.h>

using namespace Node;


FeedbackProcessorNode::FeedbackProcessorNode( PartID pID, std::string PartName ):
	fp( pID ),
	nodeName( PartName+"FeedbackProcessor" ),
	toMotionControllerFeedbackTopicName( PartName+"ProcessedFeedback" ),
	toMotionControllerFinishActionTopicName( PartName+"FinishAction" ),
	fromMotionControllerTeensyCommandTopicName( PartName+"TeensyCommand" ),
	fromTeensySubscribeTopicName( PartName+"TeensyFeedback" ),
	fromMotionControllerActionStartNotifierTopicName( PartName+"ActionState" ),
	heartrateFeedbackName( PartName+"HeartrateFeedback" ),
	valid( true )
{
	processedFeedbackPublisher = this->advertise<body_msgs::PartMsg>( toMotionControllerFeedbackTopicName, default_queue_size );
	actionEndReporter = this->advertise<support_msgs::ActionEndReporterMsg>( toMotionControllerFinishActionTopicName, default_queue_size );
	teensyCommandSubscriber = this->subscribe( fromMotionControllerTeensyCommandTopicName,
			default_queue_size,
			&Node::FeedbackProcessorNode::teensyCommandListnerCallback, this );
	currentStatePublisher = this->advertise<std_msgs::Bool>( heartrateFeedbackName, default_queue_size );
	teensyListner = this->subscribe( fromTeensySubscribeTopicName,
			default_queue_size,
			&Node::FeedbackProcessorNode::teensyListnerCallback, this );
	actionStartListner = this->subscribe( fromMotionControllerActionStartNotifierTopicName,
			default_queue_size,
			&Node::FeedbackProcessorNode::actionStartListnerCallback, this );
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& Node::FeedbackProcessorNode::publish_currentState, this ));
	fpValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind(& Node::FeedbackProcessorNode::checkFpValidness, this ));
}


void FeedbackProcessorNode::teensyCommandListnerCallback( const teensy_msgs::CommandMsg::ConstPtr & msg ){
	bool isActionFinished = fp.add_pendingScenes( msg );
	if ( isActionFinished ){
		support_msgs::ActionEndReporterMsg endReporter;
		fp.set_ActionEndReporterMsg( endReporter );
		actionEndReporter.publish( endReporter );
	}
}


void FeedbackProcessorNode::teensyListnerCallback ( const teensy_msgs::FeedbackMsg::ConstPtr & msg ){
	Body::Part processedFeedback = fp.process_Feedback( msg );
	body_msgs::PartMsg publishMsg;
	processedFeedback.set_PartMsg( publishMsg );
	processedFeedbackPublisher.publish( publishMsg );
}


void FeedbackProcessorNode::publish_currentState() {
	std_msgs::Bool msg;
	msg.data = valid;
	currentStatePublisher.publish( msg );
}


void FeedbackProcessorNode::actionStartListnerCallback( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg ) {
	fp.start_action( msg, nodeName );
}


void FeedbackProcessorNode::checkFpValidness() {
	valid = fp.isValid();
}

bool FeedbackProcessorNode::isValid() const { return valid; }


void FeedbackProcessorNode::renewAllPublisherTimer() {
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& Node::FeedbackProcessorNode::publish_currentState, this ));
}
