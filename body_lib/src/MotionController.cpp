// MotionController.cpp: implements MotionController class in Body.h
#include <body_lib/Body.h>
namespace Body{

MotionController::MotionController( PartID ID ) :
	id( ID ),
	actualCurrentState( ID ),
	valid( true )
{}

void MotionController::set_action( body_msgs::PartCommandMsg::ConstPtr & msg ) {
	if ( id != msg->id ){
        ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since message id does not match.");
        return LOCOMOTION_ACTION_ERROR;
    }

	if ( msg->sequenceSize != msg->KondoServoCommand.sequence.size() ||
		 msg->sequenceSize != msg->BrushedMotorCommand.sequence.size() ||
		 msg->sequenceSize != msg->BrushlessMotorCommand.sequence.size() ){
		ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since sequence sizes do not match.");
        return LOCOMOTION_ACTION_ERROR;
	}

	expectedStates = std::vector( msg->sequenceSize );

    for (
			std::vector<Part>::const_iterator kondoservo_iterator = msg->kondoServoCommand.begin(),
			std::vector<Part>::const_iterator brushed_iterator = msg->brushedMotorCommand.begin(),
			std::vector<Part>::const_iterator brushless_iterator = msg->brushlessMotorCommand.begin();
			servo_iterator != msg->kondoServoCommand.end();
			++kondoservo_iterator,
			++brushed_iterator,
			++brushless_iterator
		) {
        expectedStates.push_back( Part( id, *kondoservo_iterator, *brushed_iterator, *brushless_iterator ));
		if ( !expectedState.last().isValid() )
			break;
    }
	if( expectedState.size() != msg->sequenceSize ) return LOCOMOTION_ACTION_ERROR;
	return SUCCESS;
}

CattyError MotionController::init_part( init_service::PartInit::Request &,
										init_service::PartInit::Response & ){

}

CattyError MotionController::startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & ) {
	if ( expectedStates.front() == actualCurrentState ) {
		ROS_INFO("The expected position too far from the current position and thus could not start action.");
		return LOCOMOTION_ACTION_FAILUE;
	}
	timeOfActionStart = ros::Time::now();
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

void MotionController::set_CommandMsg( teensy_msgs::CommandMsg & msg ) const {
    expectedCurrentScence->set_CommandMsg( msg );
}

bool MotionController::isEnd() const {
	return expectedCurrentScene == expectedStates.end();
}

void MotionController::set_feedbackMsg( motioncontroll_action::MotionControllFeedback & feedback ) const {
	feedback.curretScene = currentScene->get_PartMsg;
	feedback.curretSceneDuration = actualSceneDuration;
}

void MotionController::set_resultMsg( motioncontroll_action::MotionControllResult & msg ) const {
	result.totalDuration = ros::Time::now() - timeOfActionStart;
	result.finalState = * actualCurrentState;
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

ros::Duration MotionController::get_actualSceneDuration() const {
	return actualSceneDuration;
}

bool MotionController::isValid() {
	return valid;
}

} // closing namespace
