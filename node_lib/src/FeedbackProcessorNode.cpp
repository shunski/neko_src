#include <node_lib/Node.h>

using namespace node;


FeedbackProcessorNode::FeedbackProcessorNode( PartID pID, std::string PartName ):
	GenericCattyNode( PartName+"FeedbackProcessor" ),
	fp( pID ),
	toMotionControllerFeedbackTopicName( PartName+"ProcessedFeedback" ),
	toMotionControllerFinishActionTopicName( PartName+"FinishAction" ),
	fromMotionControllerTeensyCommandTopicName( PartName+"TeensyCommand" ),
	fromTeensySubscribeTopicName( PartName+"TeensyFeedback" ),
	fromMotionControllerActionStartNotifierTopicName( PartName+"ActionStartNotifier" ),
	heartPumpedTopicName( nodeName+"HeartPumped" ),
	valid( true )
{
	processedFeedbackPublisher = this->advertise<body_msgs::ProcessedFeedbackMsg>( toMotionControllerFeedbackTopicName, default_queue_size );
	actionEndReporter = this->advertise<support_msgs::ActionEndReporterMsg>( toMotionControllerFinishActionTopicName, default_queue_size );
	teensyCommandSubscriber = this->subscribe( fromMotionControllerTeensyCommandTopicName,
			default_queue_size,
			&node::FeedbackProcessorNode::teensyCommandListenerCallback, this );
	currentStatePublisher = this->advertise<std_msgs::Bool>( heartPumpedTopicName, default_queue_size );
	teensyListner = this->subscribe( fromTeensySubscribeTopicName,
			default_queue_size,
			&node::FeedbackProcessorNode::teensyListenerCallback, this );
	actionStartListner = this->subscribe( fromMotionControllerActionStartNotifierTopicName,
			default_queue_size,
			&node::FeedbackProcessorNode::actionStartListenerCallback, this );
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& node::FeedbackProcessorNode::publish_currentState, this ));
	fpValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind(& node::FeedbackProcessorNode::checkFpValidness, this ));
}


void FeedbackProcessorNode::teensyCommandListenerCallback( const teensy_msgs::CommandMsg::ConstPtr & msg ){
	ROS_INFO("Message from mc_node. part_id=[%d]", msg->part_id);
	bool isActionFinished = fp.add_pendingScenes( msg );
	if ( isActionFinished ){
		Uint8 scenesLeft = fp.get_scenesLeft();
		actionEndTimer = this->createTimer( fp.get_expectedSceneDuration() * (2.0*scenesLeft), boost::bind( &node::FeedbackProcessorNode::end_action, this ), true );
	}
}


void FeedbackProcessorNode::end_action(){
	support_msgs::ActionEndReporterMsg endReporter;
	fp.set_ActionEndReporterMsg( endReporter );
	actionEndReporter.publish( endReporter );
	fp.end_action();
	ROS_INFO("Action done in Feedback Processor. Reporting the end of action to the motion controlller.");
}


void FeedbackProcessorNode::teensyListenerCallback ( const teensy_msgs::FeedbackMsg::ConstPtr & msg ){
	if( !fp.isInAction() ){
		ROS_INFO("Feedback processor Node is not in action any more. A feedback from the teensy is thus discarded.");
		return;
	}
	ROS_INFO("A feedback message of scene_id=[%d] and of part_id=[%d] is received from teensy.", msg->scene_id, msg->part_id );
	body_msgs::ProcessedFeedbackMsg publishMsg;
	fp.process_Feedback( msg, publishMsg );
	processedFeedbackPublisher.publish( publishMsg );
}


void FeedbackProcessorNode::publish_currentState() {
	std_msgs::Bool msg;
	msg.data = valid;
	currentStatePublisher.publish( msg );
}


void FeedbackProcessorNode::actionStartListenerCallback( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg ) {
	fp.start_action( msg, nodeName );
}


void FeedbackProcessorNode::checkFpValidness() {
	valid = fp.isValid();
}

bool FeedbackProcessorNode::isValid() const { return valid; }


void FeedbackProcessorNode::heartPumped() {
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& node::FeedbackProcessorNode::publish_currentState, this ));
}
