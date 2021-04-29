// MotionControllerNode.cpp implements MotionControllerNode class in Node.h
#include <node_lib/Node.h>
using namespace node;

MotionControllerNode::MotionControllerNode( PartID pID, std::string PartName ):
	GenericCattyNode( PartName+"MotionController"),
	mc( pID ),
	feedbackProcessorName( PartName + "FeedbackProcessor" ),
	fromFeedbackProcessorFeedbackTopicName( PartName+"ProcessedFeedback" ),
	fromFeedbackProcessorFinishActionTopicName( PartName+"FinishAction" ),
	toTeensyPublishTopicName( PartName+"TeensyCommand" ),
	toFeedbackProcessorActionStartNotifierName( PartName+"ActionStartNotifier" ),
	heartPumpedTopicName( nodeName+"HeartratePumped" ),
	locomotionActionName(PartName+"LocomotionAction"),
	locomotionServer( *((ros::NodeHandle*)this), locomotionActionName, boost::bind( &node::MotionControllerNode::locomotionActionCallback, this, _1 ), false )
{
	ROS_INFO("Initializing MotionControllerNode...");

	// Checking if the Feedback Processor is alive
	bool isFeedbackProcessorAlive = false;
	ros::ServiceClient fpCheckClient = this->serviceClient<support_srvs::CheckIfSpecificNodeAliveSrv>( "CheckLivenessOfNode" );
	support_srvs::CheckIfSpecificNodeAliveSrv checkSrv;
	checkSrv.request.nodeName = feedbackProcessorName;
	do {
		fpCheckClient.call( checkSrv );
		isFeedbackProcessorAlive = checkSrv.response.isAlive;
		ROS_INFO("Waiting for Feedback Processor starting.");
		ros::Duration( 0.1 ).sleep();
	} while ( !isFeedbackProcessorAlive );

	// initializing various stuff
    locomotionServer.start();
	valid = mc.isValid();
    commandPublisher = this->advertise<teensy_msgs::CommandMsg>( toTeensyPublishTopicName, default_queue_size );
    actionStartNotifier = this->advertise<support_msgs::ActionStartNotifierMsg>( toFeedbackProcessorActionStartNotifierName, 10 );
	actionEndListner = this->subscribe( fromFeedbackProcessorFinishActionTopicName,
			default_queue_size,
			& node::MotionControllerNode::end_action, this );
    currentStatePublisher = this->advertise<std_msgs::Bool>( heartPumpedTopicName, default_queue_size);
    feedbackProcessorListner = this->subscribe( fromFeedbackProcessorFeedbackTopicName,
			default_queue_size,
			& node::MotionControllerNode::processorListenerCallback, this );
    mcValidnessSensor = this->createTimer( ros::Duration(0.5), boost::bind( &node::MotionControllerNode::checkMcValidness, this ));
	ROS_INFO("Done.");
}


void MotionControllerNode::heartPumped () {

}


void MotionControllerNode::end_action( const support_msgs::ActionEndReporterMsg::ConstPtr & msg ){
	ROS_INFO("End of action Reported by the Feedback Processor.");
    mc.update_locomotionActionResultMsg( msg, nodeName );
	mc.end_action();
}


void MotionControllerNode::processorListenerCallback ( const body_msgs::ProcessedFeedbackMsg::ConstPtr & msg ){
	mc.update_locomotionActionFeedbackMsg( msg, nodeName );
}


void MotionControllerNode::locomotionActionCallback ( const motioncontroll_action::MotionControllGoal::ConstPtr & goal )
{
	ROS_INFO("Decoding Action.");
    CattyError error = mc.set_action( goal );
    if ( error == SUCCESS ){
		ROS_INFO("Action successfully set up. Starting Action.");
	} else {
		ROS_ERROR("Unable to set up the action at motion controller node. Quitting action.");
        locomotionServer.setPreempted();
		mc.end_action();
        return;
    }

    support_msgs::ActionStartNotifierMsg notifierMsg;
    mc.set_actionStartNotifier( notifierMsg );
    actionStartNotifier.publish( notifierMsg );
	ros::Duration(1.0/100).sleep();

	ros::Timer publishFeedbackTimer = this->createTimer( mc.get_expectedSceneDuration()*0.5, boost::bind( &node::MotionControllerNode::publish_feedback, this ));

	mc.startMotioncontrollAction();

    while( !mc.isEnd() ){
        if ( locomotionServer.isPreemptRequested() || !ros::ok() ){
            ROS_INFO("%s: Preempted Requested", locomotionActionName.c_str());
            locomotionServer.setPreempted();
			mc.end_action();
            break;
        }

        publish_CommandMsg();
        mc.proceed();
    }

    while( mc.isInAction()) ros::Rate( 100 ).sleep();

    ROS_INFO("%s Succeeded", locomotionActionName.c_str());
    locomotionServer.setSucceeded( mc.get_locomotionActionResultMsg() );
}


void MotionControllerNode::publish_CommandMsg() const
{
	teensy_msgs::CommandMsg publishMsg;
	mc.set_CommandMsg( publishMsg );
    commandPublisher.publish( publishMsg );
}


void MotionControllerNode::publish_feedback()
{
	motioncontroll_action::MotionControllFeedback feedback;
	bool isNewFeedbackReceived = mc.set_locomotionActionFeedbackMsg( feedback );
    if ( isNewFeedbackReceived )
    	locomotionServer.publishFeedback( feedback );
}


void MotionControllerNode::publish_currentState() const
{
    std_msgs::Bool publishMsg;
	publishMsg.data = valid;
	currentStatePublisher.publish( publishMsg );
}


void MotionControllerNode::checkMcValidness() {
	valid = mc.isValid();
	if( !valid ) {
		ROS_ERROR("This Motion Controller Node is invalid. Shutting down.");
		ros::shutdown();
	}
}


bool MotionControllerNode::isValid() const { return valid; }
