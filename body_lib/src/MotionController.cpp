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
        return LOCOMOTION_ACTION_FAILUE;
    }

	if ( msg->command.sequenceSize != msg->command.kondoServoCommandSequence.size() ||
		 msg->command.sequenceSize != msg->command.brushedMotorCommandSequence.size() ||
		 msg->command.sequenceSize != msg->command.brushlessMotorCommandSequence.size() ){
		ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since the message is not well defined; sizes do not match.");
        return LOCOMOTION_ACTION_FAILUE;
	}

	expectedStates = std::vector<Part>( msg->command.sequenceSize );

    for (
			auto t = std::make_tuple( 
			std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator( msg->command.kondoServoCommandSequence.begin() ),
			std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator( msg->command.brushedMotorCommandSequence.begin()),
			std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator( msg->command.brushlessMotorCommandSequence.begin())
			);
			std::get<std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator>(t) = msg->command.kondoServoCommandSequence.end();
			++std::get<std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator>(t),
			++std::get<std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator>(t),
			++std::get<std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator>(t)
		) {
        expectedStates.push_back( 
			Part( 
				part_id, 
				* std::get<std::vector<parts_msgs::KondoServoCommandMsg>::const_iterator>(t), 
				* std::get<std::vector<parts_msgs::BrushedMotorCommandMsg>::const_iterator>(t), 
				* std::get<std::vector<parts_msgs::BrushlessMotorCommandMsg>::const_iterator>(t)
			)
		);
		if ( !expectedStates.back().isValid() )
			break;
    }
	if( expectedStates.size() != msg->command.sequenceSize ) return LOCOMOTION_ACTION_FAILUE;
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

CattyError MotionController::set_feedbackMsg( motioncontroll_action::MotionControllFeedback & feedback ) const {
    if ( part_id != feedback.part_id && feedback.part_id != 0 ){
        ROS_INFO("ERROR: Could not set [motioncontroll_action::MotionControllFeedback] message because id does not match.");
        return PART_ID_NOT_MATCH;
    }
	feedback.actualCurrentState = actualCurrentState.get_PartMsg();
	feedback.currentSceneDuration = actualCurrentSceneDuration;
	return SUCCESS;
}

CattyError MotionController::set_resultMsg( motioncontroll_action::MotionControllResult & result ) const {
    if ( part_id != result.part_id && result.part_id != 0 ){
        ROS_INFO("ERROR: Could not set [motioncontroll_action::MotionControllResult] message because part_id does not match.");
        return PART_ID_NOT_MATCH;
    }

	result.actualTotalDuration = ros::Time::now() - timeOfActionStart;
	result.actualFinalState = actualCurrentState.get_PartMsg();
	return SUCCESS;
}

motioncontroll_action::MotionControllFeedback MotionController::generate_feedbackMsg() const {
    motioncontroll_action::MotionControllFeedback feedback;
    feedback.actualCurrentState = actualCurrentState.get_PartMsg();
    feedback.currentSceneDuration = actualCurrentSceneDuration;
    return feedback;
}

motioncontroll_action::MotionControllResult MotionController::generate_resultMsg() const {
    motioncontroll_action::MotionControllResult result;
    result.actualTotalDuration = ros::Time::now() - timeOfActionStart;
    result.actualFinalState = actualCurrentState.get_PartMsg();
    return result;
}

ros::Duration MotionController::get_actualCurrentSceneDuration() const {
	return actualCurrentSceneDuration;
}

ros::Duration MotionController::get_expectedSceneDuration() const {
	return expectedSceneDuration;
}

bool MotionController::isValid() {
	return valid;
}

} // closing namespace
