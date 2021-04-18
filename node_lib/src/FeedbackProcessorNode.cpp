#include <ros/ros.h>
#include <node_lib/Node.h>

using namespace Node;


FeedbackProcessorNode(PartID ID, std::string PartName )
	fp( ID ),
	nodeName( PartName+"FeedbackProcessor" ),
	toMotionControllerPublishTopicName( nodeName+"processedFeedback" ),
	fromTeensySubscribeTopicName( nodeName+"TeensyFeedback" ),
	fromMotionControllerServiceTopicName( nodeName+"ActionState" ),
	heartrateFeedbackName( nodeName+"HeartrateFeedback" ),
	valid( true )
{
	processedFeedbackPublisher = this->advertise<body_msgs::PartMsg>(ToMotionControllerPublishTopicName, queue_size );
	currentStatePublisher = this->advertise<std_msgs::bool>( heartrateFeedbackName, queue_size );
	teensyListner = this->advertise<teensy_msgs::FeedbackMsg>( FromTeensySubscribeTopicName, queue_size,
                                                                boost::bind( &Node::FeedbackProcessorNode::teensyListerCallback, this ));
	actionStateServer = this->advertiseService<support_srv::LocomotionActionStateSrv>( FromTeensySubscribeTopicName,
                                                boost::bind( &Node::FeedbackProcessorNode::locomotionActionStateServerCallback, this ));
	currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& Node::FeedbackProcessorNode::publish_currentState(), this ));
	fpValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind(& Node::FeedbackProcessorNode::checkFpValidness(), this ))
}


void FeedbackProcessorNode::teencyListnerCallback ( teensy_msgs::FeedbackMsg::ConstPtr msg ){
	Part processedFeedback = fp.processFeedback( msg );
	PartMsg publishingMsg;
	processFeedback.set_PartMsg( publishingMsg );
	publish_processedFeedback( msg );
}


inline void FeedbackProcessorNode::publish_processedFeedback( const body_msgs::PartMsg & ) const {
	processedFeedbackPublisher.publish( msg );
}


void FeedbackProcessorNode::publish_currentState() {
	std_msgs::bool msg = valid;
	currentStatePublisher.publish( msg );
}


bool FeedbackProcessorNode::locomotionActionStateServerCallback( support_srvs::LocmotionActionStateSrv::Request & req
	                                                             support_srvs::LocmotionActionStateSrv::Response & res ){
	fp.start_action( rep.sequenceSize, nodeName );

	ros::Rate rate(10);
	while( fp.isInAction() ){
		rate.sleep();
	}

	if ( fp.isValid() ){
		fp.set_stateOfScenes( res );
		ROS_INFO("%s: Action Done. Returning the response.");
		return true;
	} else {
		ROS_INFO("Error on %s: Action Not Complete.");
		return true;
	}
}


void FeedbackProcessorNode::checkFpValidness(){
	valid = fp.isValid();
}
