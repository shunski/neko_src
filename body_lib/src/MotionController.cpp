// MotionController.cpp: implements MotionController class in Body.h
#include <body_lib/Body.h>
namespace Body{


MotionController::MotionController( PartID ID ):
	part_id( ID ),
	actualCurrentState( ID ),
	valid( true )
{}


CattyError MotionController::set_action( motioncontroll_action::MotionControllGoal::ConstPtr & msg ) {
	if ( part_id != msg->part_id ){
        ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since message id does not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILUE;
    }

	if ( msg->sequenceSize != msg->partCommandSequence.size() ){
		ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since the message is not well defined; sizes do not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILUE;
	}

	expectedStates = std::vector<Part>( msg->sequenceSize );

    for ( std::vector<body_msgs::PartCommandMsg>::const_iterator it = msg->partCommandSequence.begin();
			it != msg->partCommandSequence.end();
			++it ) {
        expectedStates.push_back( 
			Part( 
				part_id, 
				it->kondoServoCommandSet,
				it->brushedMotorCommandSet,
				it->brushlessMotorCommandSet
			)
		);
		if ( !expectedStates.back().isValid() )
			break;
    }
	if( expectedStates.size() != msg->sequenceSize ) return LOCOMOTION_ACTION_FAILUE;
	return SUCCESS;
}


// CattyError MotionController::init_part( init_service::PartInit::Request &,
// 										init_service::PartInit::Response & ){
// 
// };

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


CattyError MotionController::set_CommandMsg( teensy_msgs::CommandMsg & msg ) const {
	if ( part_id != msg.part_id && msg.part_id != 0 ){
		ROS_INFO("ERROR: Could not set [teensy_msgs::CommandMsg] message because id does not match.");
		return PART_ID_NOT_MATCH;
	}
    expectedCurrentState->set_CommandMsg( msg );
	return SUCCESS;
}


bool MotionController::isEnd() const {
	return expectedCurrentState == expectedStates.end();
}


CattyError MotionController::set_feedbackMsg( motioncontroll_action::MotionControllFeedback & feedback ) {
    if ( part_id != feedback.part_id && feedback.part_id != 0 ){
        ROS_INFO("ERROR: Could not set [motioncontroll_action::MotionControllFeedback] message because id does not match.");
        return PART_ID_NOT_MATCH;
    }
	body_msgs::PartMsg actualCurrentStateMsg;
	CattyError error = actualCurrentState.set_PartMsg( actualCurrentStateMsg );
	if ( error != SUCCESS ) {
		ROS_INFO("Terminating program since seirous error occured: %s", get_catty_error_description( error ).c_str());
		valid = false;
		return error;
	}
	feedback.actualCurrentState = actualCurrentStateMsg;
	feedback.currentSceneDuration = actualCurrentSceneDuration;
	return SUCCESS;
}


CattyError MotionController::set_resultMsg( motioncontroll_action::MotionControllResult & result ) {
    if ( part_id != result.part_id && result.part_id != 0 ){
        ROS_INFO("ERROR: Could not set [motioncontroll_action::MotionControllResult] message because part_id does not match.");
        return PART_ID_NOT_MATCH;
    }

	body_msgs::PartMsg actualFinalStateMsg;
	CattyError error = actualCurrentState.set_PartMsg( actualFinalStateMsg );

	if ( error != SUCCESS ){
		ROS_INFO("Terminating MotionController since seirous error occured: %s", get_catty_error_description( error ).c_str());
		valid = false;
		return error;
	}

	result.actualFinalState = actualFinalStateMsg;
	result.actualTotalDuration = ros::Time::now() - timeOfActionStart;
	return SUCCESS;
}


ros::Duration MotionController::get_actualCurrentSceneDuration() const {
	return actualCurrentSceneDuration;
}


ros::Duration MotionController::get_expectedSceneDuration() const {
	return expectedSceneDuration;
}


bool MotionController::isValid() const { return valid; }


} // closing namespace
