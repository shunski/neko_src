// MotionControllerNode.cpp implements MotionControllerNode class in Node.h
#include <jcatty_support_lib/Node.h>
using namespace Node;

MotionControllerNode::MotionControllerNode(std::string publishCmdTopicName, std::string publishStateTopicName, std::string subscribeInfoTopicName, std::string locomotionActionName):
    locomotionServer(this, locomotionActionName, boost::bind(& Node::MotionControllerNode::locomotionActionCallback, this), false)
{
    locomotionServer.start();
    legCmdPublisher = this->advertise<jcatty_teensy_msgs::CommandMsg>(publishCmdTopicName, queue_size);
    currentStatePublisher = this->advertise<jcatty_body_msgs::PartMsg>(publishStateTopicName, queue_size);
    legInfoProcessorListner = this->subscribe<jcatty_body_msgs::PartMsg>(subscribeInfoTopicName,
                                                                  queue_size,
                                                                  boost::bind(& Node::MotionControllerNode::processorListnerCallback, this));
    publisherTimer = this->createTimer(heartrate, boost::bind(& Node::HindlegCmdPublisher::publishCmd()), this);
    actionName = locomotionActionName;
}

void MotionControllerNode::renewAllPublisherTimer () override {
    publisherTimer = this->createTimer(heartrate, boost::bind(& Node::McNode::publish_currentState()), this);
}

void MotionControllerNode::processorListnerCallback ( jcatty_body_msgs::PartMsg::ConstPtr msg ){
    mc.reflectFeedback(msg);
}

void MotionControllerNode::locomotionActionCallback ( const jcatty_locomotion_action::MotionControllAction::ConstPtr & goal) {
    bool success;
    ros::Duration sleepPeriod = mc.get_sceneDuration() / 2.0;
    CmdMsg cmdMsg;
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
        jcatty_locomotion_action::MotionControllResult result = mc.get_resultMsg();
        ROS_INFO("%s Succeeded", actionName.c_str());
        setSucceed(result);
    }
}

inline void MotionControllerNode::publish_cmdMsg() const {
    jcatty_teensy_msgs::CommandMsg msg = mc.get_CommandMsg();
    currentStatePublisher.publish( msg );
}

inline void MotionControllerNode::publish_feedback() const {
    jcatty_locomotion_action::MotionControllFeedback feedback = mc.get_feedbackMsg();
    locomotionServer.currentStatePublisher.publish( msg );
}

void MotionControllerNode::publish_currentState() const {
    jcatty_body_msgs::PartMsg msg = mc.get_currentState().set_PartMsg();
    currentStatePublisher.publish( msg );
}
