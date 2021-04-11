// MotionControllerNode.cpp implements MotionControllerNode class in Node.h
#include <node_lib/Node.h>
using namespace Node;

MotionControllerNode::MotionControllerNode( PartID ID, std::string publishCommandTopicName, std::string publishStateTopicName, std::string subscribeFeedbackTopicName, std::string locomotionActionName):
    mc( ID ),
	locomotionServer(this, locomotionActionName, boost::bind(& Node::MotionControllerNode::locomotionActionCallback, this), false)
{
    valid = mc.isValid();
    locomotionServer.start();
    legCmdPublisher = this->advertise<teensy_msgs::CommandMsg>(publishCmdTopicName, queue_size);
    currentStatePublisher = this->advertise<body_msgs::PartMsg>(publishStateTopicName, queue_size);
    legInfoProcessorListner = this->subscribe<body_msgs::PartMsg>(subscribeInfoTopicName,
                                                                  queue_size,
                                                                  boost::bind(& Node::MotionControllerNode::processorListnerCallback, this));
    publisherTimer = this->createTimer( heartrate, boost::bind(& Node::MotionControllerNode::publish_CommandMsg()), this );
    actionName = locomotionActionName;
}

void MotionControllerNode::renewAllPublisherTimer () override {
    publisherTimer = this->createTimer(heartrate, boost::bind(& Node::McNode::publish_currentState()), this);
}

void MotionControllerNode::processorListnerCallback ( body_msgs::PartMsg::ConstPtr msg ){
    mc.reflectFeedback(msg);
}

void MotionControllerNode::locomotionActionCallback ( const motioncontroll_action::MotionControllAction::ConstPtr & goal) {
    bool success;
    ros::Duration sleepPeriod = mc.get_sceneDuration() / 2.0;
    CommandMsg cmdMsg;
    mc.startAction();

    while( !mc.isEnd() ){
        if ( actionServer.isPreemptRequested() || !ros::ok() ){
            ROS_INFO("%s: Preempted Requested", actionName.c_str());
            success = false;
            actionName.setPreempted();
            break;
        }
        publish_cmdMsg();
        sleepPeriod.sleep();
        publish_feedback();
        mc.proceed();
    }

    if ( success ) {
        motioncontroll_action::MotionControllResult result = mc.get_resultMsg();
        ROS_INFO("%s Succeeded", actionName.c_str());
        setSucceed(result);
    }
}

inline void MotionControllerNode::publish_CommandMsg() const {
    teensy_msgs::CommandMsg msg = mc.get_CommandMsg();
    currentStatePublisher.publish( msg );
}

inline void MotionControllerNode::publish_feedback() const {
    motioncontroll_action::MotionControllFeedback feedback = mc.get_feedbackMsg();
    locomotionServer.currentStatePublisher.publish( msg );
}

void MotionControllerNode::publish_currentState() const {
    body_msgs::PartMsg msg = mc.get_currentState().set_PartMsg();
    currentStatePublisher.publish( msg );
}
