// MotionControllerNode.cpp implements MotionControllerNode class in Node.h
#include <node_lib/Node.h>
using namespace Node;

MotionControllerNode::MotionControllerNode( PartID pID, std::string PartName ):
    mc( pID ),
    nodeName( PartName+"MotionController"),
    fromFeedbackProcessorFeedbackTopicName( PartName+"controlledMotion" ),
    fromFeedbackProcessorFinishActionTopicName( PartName+"FinishAction" ),
    toTeensyPublishTopicName( PartName+"TeensyCommand" ),
    toFeedbackProcessorActionStartNotifierName( PartName+"ActionStartNotifier" ),
    heartrateFeedbackName( PartName+"HeartrateFeedback" ),
	locomotionServer( *(ros::NodeHandle*)(this), locomotionActionName, boost::bind( &Node::MotionControllerNode::locomotionActionCallback, this, _1 ), false )
{
    locomotionServer.start();
	valid = mc.isValid();
    commandPublisher = this->advertise<teensy_msgs::CommandMsg>( toTeensyPublishTopicName, default_queue_size );
    actionStartNotifier = this->advertise<support_msgs::ActionStartNotifierMsg>( toFeedbackProcessorActionStartNotifierName, 10 );
    actionEndListner = this->subscribe( fromFeedbackProcessorFinishActionTopicName,
			default_queue_size,
			& Node::MotionControllerNode::end_action, this );
    currentStatePublisher = this->advertise<std_msgs::Bool>( heartrateFeedbackName, default_queue_size);
    feedbackProcessorListner = this->subscribe( fromFeedbackProcessorFeedbackTopicName,
			default_queue_size,
			& Node::MotionControllerNode::processorListnerCallback, this );
    currentStatePublisherTimer = this->createTimer( heartrate, boost::bind( &Node::MotionControllerNode::publish_CommandMsg, this ));
    mcValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind( &Node::MotionControllerNode::checkMcValidness, this ));
}


void MotionControllerNode::renewAllPublisherTimer () {
    currentStatePublisherTimer = this->createTimer(heartrate, boost::bind( &Node::MotionControllerNode::publish_currentState, this ));
}


void MotionControllerNode::end_action( const support_msgs::ActionEndReporterMsg::ConstPtr & msg ){
    mc.update_locomotionActionResultMsg( msg, nodeName );
}


void MotionControllerNode::processorListnerCallback ( const body_msgs::PartMsg::ConstPtr & msg ){
    mc.add_processedFeedbackPending( msg );
}


void MotionControllerNode::locomotionActionCallback ( const motioncontroll_action::MotionControllGoal::ConstPtr & goal )
{
    bool success;
    CattyError error = mc.set_action( goal );
    if ( error != SUCCESS )
    {
        locomotionServer.setPreempted();
        return;
    }

    ros::Duration commandDuration = mc.get_expectedSceneDuration();

    support_msgs::ActionStartNotifierMsg notifierMsg;
    mc.set_actionStartNotifier( notifierMsg );
    actionStartNotifier.publish( notifierMsg );

    mc.set_action( goal );

    while( !mc.isEnd() ){
        if ( locomotionServer.isPreemptRequested() || !ros::ok() ){
            ROS_INFO("%s: Preempted Requested", locomotionActionName.c_str());
            success = false;
            locomotionServer.setPreempted();
            break;
        }

        publish_CommandMsg();
        commandDuration.sleep();
        publish_feedback();
        mc.proceed();
    }

    ros::Rate rate( 10 );
    while( mc.isInAction() ){
        rate.sleep();
    }

    if ( success ) {
        ROS_INFO("%s Succeeded", locomotionActionName.c_str());
        locomotionServer.setSucceeded( mc.get_locomotionActionResultMsg() );
    }
}


void MotionControllerNode::publish_CommandMsg() const
{
	teensy_msgs::CommandMsg publishMsg;
	mc.set_CommandMsg( publishMsg );
    currentStatePublisher.publish( publishMsg );
}


void MotionControllerNode::publish_feedback() 
{
    motioncontroll_action::MotionControllFeedback feedback = mc.get_locomotionActionFeedbackMsg();
    locomotionServer.publishFeedback( feedback );
}


void MotionControllerNode::publish_currentState() const
{
    std_msgs::Bool publishMsg;
	publishMsg.data = valid;
	currentStatePublisher.publish( publishMsg );
}


void MotionControllerNode::checkMcValidness() { valid = mc.isValid(); }


bool MotionControllerNode::isValid() const { return valid; }
