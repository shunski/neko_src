// MotionController.cpp: implements MotionController class in Body.h
#include <body_lib/Body.h>
namespace Body{

MotionController::MotionController( PartID ID ):
	id( ID ),
	actualCurrentState( ID ),
	valid( true )
{}

CattyError MotionController::set_action( body_msgs::PartCommandMsg::ConstPtr & msg ) {
	if ( id != msg->id ){
        ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since message id does not match.");
        return LOCOMOTION_ACTION_ERROR;
    }

	if ( msg->command.sequenceSize != msg->command.kondoServoCommand.sequence.size() ||
		 msg->command.sequenceSize != msg->command.brushedMotorCommand.sequence.size() ||
		 msg->command.sequenceSize != msg->command.brushlessMotorCommand.sequence.size() ){
		ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since sequence sizes do not match.");
        return LOCOMOTION_ACTION_ERROR;
	}

	expectedStates = std::vector<Part>( msg->command.sequenceSize );

    for (
			std::vector<Part>::const_iterator kondoservo_iterator = msg->command.kondoServoCommand.begin(),                                //forの区切り方？？
			std::vector<Part>::const_iterator brushed_iterator = msg->command.brushedMotorCommand.begin(),
			std::vector<Part>::const_iterator brushless_iterator = msg->command.brushlessMotorCommand.begin();
			kondoservo_iterator != msg->kondoServoCommand.end();
			++kondoservo_iterator,
			++brushed_iterator,
			++brushless_iterator
		) {
        expectedStates.push_back( Part( id, *kondoservo_iterator, *brushed_iterator, *brushless_iterator ));
		if ( !expectedStates.last().isValid() )
			break;
    }
	if( expectedStates.size() != msg->command.sequenceSize ) return LOCOMOTION_ACTION_ERROR;
	return SUCCESS;
}

CattyError MotionController::init_part( init_service::PartInit::Request &,                  // 全体的にエラー
										init_service::PartInit::Response & ){

};

// CattyError MotionController::startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & ) {
// 	if ( expectedStates.front() == actualCurrentState ) {
// 		ROS_INFO("The expected position too far from the current position and thus could not start action.");
// 		return LOCOMOTION_ACTION_FAILUE;
// 	}
// 	timeOfActionStart = ros::Time::now();
// }

void MotionController::procced() {
    if (expectedCurrentState == expectedStates.begin()){ //first case
        ros::Time currentTime = ros::Time::now();
        actualCurrentSceneDuration = currentTime - timeOfActionStart;
        timeOfLastAction = currentTime;
    }
    else{
        ros::Time currentTime = ros::Time::now();
        actualCurrentSceneDuration = currentTime - timeOfLastAction;
        timeOfLastAction = currentTime;
    }
    ++expectedCurrentState;
}

void MotionController::set_CommandMsg( teensy_msgs::CommandMsg & msg ) const {
    expectedCurrentState->set_CommandMsg( msg );
}

bool MotionController::isEnd() const {
	return expectedCurrentState == expectedStates.end();
}

void MotionController::set_feedbackMsg( motioncontroll_action::MotionControllFeedback & feedback ) const {
	feedback.actualCurrentState = actualCurrentState->get_PartMsg;            // ->
	feedback.currentSceneDuration = actualCurrentSceneDuration;
}

void MotionController::set_resultMsg( motioncontroll_action::MotionControllResult & result ) const {
	result.totalDuration = ros::Time::now() - timeOfActionStart;
	result.finalState = * actualCurrentState;                                 //finalState is not member   //no match *
}

motioncontroll_action::MotionControllFeedback MotionController::get_feedbackMsg() const {
    motioncontroll_action::MotionControllFeedback feedback;
    feedback.actualCurrentState = actualCurrentState->get_PartMsg;          // ->
    feedback.currentSceneDuration = actualCurrentSceneDuration;
    return feedback;
}

motioncontroll_action::MotionControllResult MotionController::get_resultMsg() const {
    motioncontroll_action::MotionControllResult result;
    result.totalDuration = ros::Time::now() - timeOfActionStart;
    result.actualFinalState = actualCurrentState;                                  //finalState is not member   //no match *
    return result;
}

ros::Duration MotionController::get_actualCurrentSceneDuration() const {
	return actualCurrentSceneDuration;                                        //cannot return
}

ros::Duration get_expectedSceneDuration() const {
	return expectedSceneDuration;                                             //cannot return
}

bool MotionController::isValid() {
	return valid;
}

} // closing namespace
