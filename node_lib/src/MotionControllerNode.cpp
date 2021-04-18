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
	locomotionServer(this, locomotionActionName, boost::bind(& Node::MotionControllerNode::locomotionActionCallback, this), false),
    valid( ture )
{
    valid = mc.isValid();
    locomotionServer.start();
    commandPublisher = this->advertise<teensy_msgs::CommandMsg>( toTeensyPublishTopicName, queue_size);
    actionStartNotifier = this->advertise<support_msgs::actionStartNotifierMsg>( toFeedbackProcessorActionStartNotifierName, 10 );
    actionEndListner = this->subscribe<support_msgs::ActionEndReporterMsg>( fromFeedbackProcessorFinishActionTopicName,
                                                                            boost::bind(& Node::MotionContollerNode::end_action, this));
    currentStatePublisher = this->advertise<std_msgs::bool>( heartrateFeedbackName, queue_size);
    feedbackProcessorListner = this->subscribe<body_msgs::PartMsg>( fromFeedbackProcessorSubscribeTopicName,
                                                                   queue_size,
                                                                   boost::bind(& Node::MotionControllerNode::processorListnerCallback, this));
    currentStatePublisherTimer = this->createTimer( heartrate, boost::bind(& Node::MotionControllerNode::publish_CommandMsg()), this );
    mcValidnessSensor = this->createTimer( ros::Duration(0.1), boost::bind(& Node::MotionControllerNode::checkMcValidness()), this );
}


void MotionControllerNode::renewAllPublisherTimer () {
    publisherTimer = this->createTimer(heartrate, boost::bind( & Node::MotionContollerNode::publish_currentState()), this );
}


void end_action( const support_msgs::ActionEndReporterMsg::ConstPtr & msg ){
    mc.set_resultMsg( msg );
}


void MotionControllerNode::processorListnerCallback ( const body_msgs::PartMsg::ConstPtr & msg ){
    mc.add_processedFeedbackPending( msg );
}


void MotionControllerNode::locomotionActionCallback ( const motioncontroll_action::MotionControllAction::ConstPtr & goal )
{
    bool success;
    CattyError error = mc.set_action( goal );
    if ( error != SUCCESS )
    {
        actionServer.setPreempted();
        return;
    }

    ros::Duration commandDuration = mc.get_sceneDuration();

    support_msgs::actionStartNotifierMsg notifierMsg;
    mc.set_actionStartNotifier( notifierMsg );
    actionStartNotifier.publish( notifierMsg );

    mc.startAction();

    while( !mc.isEnd() ){
        if ( actionServer.isPreemptRequested() || !ros::ok() ){
            ROS_INFO("%s: Preempted Requested", locomotionActionName.c_str());
            success = false;
            actionServer.setPreempted();
            break;
        }

        publish_CommandMsg();
        sleepPeriod.sleep();
        publish_feedback();
        mc.proceed();
    }

    ros::Rate rate( 10 );
    while( mc.isInAction() ){
        rate.sleep();
    }

    if ( success ) {
        motioncontroll_action::MotionControllResult result = mc.get_resultMsg();
        ROS_INFO("%s Succeeded", actionName.c_str());
        setSucceed( result );
    }
}


inline void MotionControllerNode::publish_CommandMsg() const
{
    currentStatePublisher.publish( mc.get_CommandMsg() );
}


inline void MotionControllerNode::publish_feedback() const
{
    motioncontroll_action::MotionControllFeedback feedback = mc.get_feedbackMsg();
    locomotionServer.currentStatePublisher.publish( msg );
}


void MotionControllerNode::publish_currentState() const
{
    body_msgs::PartMsg msg = mc.get_currentState().set_PartMsg();
    currentStatePublisher.publish( msg );
}


void MotionControllerNode::checkMcValidness() { valid = mc.isValid(); }
