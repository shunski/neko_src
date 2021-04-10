// MotionController.cpp: implements MotionController class in Body.h
#include <body_lib/Body.h>
namespace Body{

MotionController::MotionController ( PartID ID ) :
	id( ID ),
	actualCurrentState( ID ),
	isValid(true)
{}

void MotionController::set( body_msgs::PartCommandMsg::ConstPtr & msg ) {
	if ( id != msg->id ){
        ROS_INFO("ERROR: Could not set MotionController object since message id does not match.");
        return;
    }

	if ( msg->KondoServoCommand.sequence.size != msg->BrushedMotorCommand.size ||  msg->BrushedMotorCommand.size != msg->BrushlessMotorCommand.size ){
		ROS_INFO("ERROR: Could not set MotionController object since sequence sizes do not match.");
        return;
	}

    for (
			std::vector<Part>::const_iterator servo_iterator = msg->kondoServoCommand.begin(),
			std::vector<Part>::const_iterator brushed_iterator = msg->brushedMotorCommand.begin(), 
			std::vector<Part>::const_iterator brushless_iterator = msg->brushlessMotorCommand.begin();
			servo_iterator != msg->kondoServoCommand.end();
			++servo_iterator,
			++brushed_iterator,
			++brushless_iterator
		){
        Part(id, )
    }
}

void MotionController::initializePosition() {
    expectedStates.front();
}

void MotionController::procced() {
    if (expectedCurrentScene == expectedStates.begin()){ //first case
        ros::Time currentTime = ros::Time::now();
        actualSceneDuration = currentTime - timeOfActionStart;
        timeOfLastAction = currentTime;
    }
    else{
        ros::Time currentTime = ros::Time::now();
        actualSceneDuration = currentTime - timeOfLastAction;
        timeOfLastAction = currentTime;
    }
    ++expectedCurrentScene;
}

void MotionController::set_CommandMsg( teensy_msgs::CommandMsg & msg ){
    expectedCurrentScence->set_CommandMsg(msg);
}

motioncontroll_action::MotionControllFeedback MotionController::get_feedbackMsg() const {
    motioncontroll_action::MotionControllFeedback feedback;
    feedback.currentScence = currentScence->get_PartMsg();
    feedback.currentSceneDuration = actualSceneDuration;
    return feedback;
}

motioncontroll_action::MotionControllResult MotionController::get_resultMsg() const {
    motioncontroll_action::MotionControllResult result;
    result.totalDuration = ros::Time::now() - timeOfActionStart;
    result.finalState = * actualCurrentState;
    return result;
}

} // closing namespace
