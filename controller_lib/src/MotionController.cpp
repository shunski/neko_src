// MotionController.cpp: implements MotionController class in Body.h
#include "body_lib/Body.h"
using namespace body;


MotionController::MotionController( PartID ID ):
	part_id( ID ),
	valid( true ),
	inAction( false )
{
	ROS_INFO("Initializing MotionController Done.");
}


CattyError MotionController::set_action( const motioncontroll_action::MotionControllGoal::ConstPtr & msg ) {
	if ( part_id != msg->part_id ){
        ROS_ERROR("Could not set MotionController from motioncontroll_action::MotionControllGoal since message id does not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
    }

	if ( msg->sequenceSize == 0 ){
		ROS_ERROR("Could not set MotionController from motioncontroll_action::MotionControllGoal since this is an invalid action.");
        return WARNING;
	}

	if ( msg->sequenceSize != msg->partCommandSequence.size() ){
		ROS_ERROR("Could not set MotionController from motioncontroll_action::MotionControllGoal since the message is not well defined; sizes do not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
	}

	if ( inAction ) {
		ROS_ERROR("Could not set MotionController from motioncontroll_action::MotionControllGoal since another action is in progress.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
	}
	inAction = true;

	sequenceSize = msg->sequenceSize;
	expectedSceneDuration = msg->expectedTotalDuration * ( 1.0 / msg->sequenceSize );

	reset();
	expectedStates.reserve( sequenceSize );

	int sceneId = 0;
    for ( std::vector<body_msgs::PartCommandMsg>::const_iterator it = msg->partCommandSequence.begin();
			it != msg->partCommandSequence.end();
			++it ) {
        expectedStates.push_back(
			Part(
				sceneId,
				part_id,
				it->kondoServoCommandSet,
				it->brushedMotorCommandSet,
				it->brushlessMotorCommandSet
			)
		);
		sceneId++;
		if ( !expectedStates.back().isValid() )
			break;
    }
	if( expectedStates.size() != sequenceSize ) {
		ROS_ERROR("The size of expectedStates does not match to the sequence size of the message. (expectedStates.size()=%ld while msg->sequenceSize=%d)", expectedStates.size(), msg->sequenceSize );
		return LOCOMOTION_ACTION_FAILURE;
	}

	expectedCurrentState = expectedStates.begin();

	return SUCCESS;
}

void MotionController::startMotioncontrollAction() {
	timeOfActionStart = ros::Time::now();
}


void MotionController::initialize_feedbackMsg(){
	while( !motionControllFeedbacks.empty() )
		motionControllFeedbacks.pop();
}


void MotionController::initialize_resultMsg(){
	motionControllResult = motioncontroll_action::MotionControllResult();
	motionControllResult.part_id = part_id;
}


void MotionController::proceed() {
	expectedSceneDuration.sleep();
	ros::Time currentTime = ros::Time::now();
    ++expectedCurrentState;
}


CattyError MotionController::set_CommandMsg( teensy_msgs::CommandMsg & msg ) const {
	if ( part_id != msg.part_id && msg.part_id != 0 ){
		ROS_ERROR("PART_ID_NOT_MATCH: Could not set [teensy_msgs::CommandMsg] message because part_id does not match.");
		return PART_ID_NOT_MATCH;
	}
    expectedCurrentState->set_CommandMsg( msg );
	return SUCCESS;
}


bool MotionController::isEnd() const {
	return expectedCurrentState == expectedStates.end();
}


CattyError MotionController::update_locomotionActionFeedbackMsg( const body_msgs::ProcessedFeedbackMsg::ConstPtr & msg, const std::string & nodeName ){
	if ( part_id != msg->processedPart.part_id ){
		ROS_ERROR("PART_ID_NOT_MATCH(%s): Could not set the feedbackMsg of Action by body_msgs::PartMsg since PartID not match", nodeName.c_str());
		return PART_ID_NOT_MATCH;
	}

	while ( currentSceneIdProcessed < msg->processedPart.scene_id ){
		currentSceneIdProcessed++;
	}

	motioncontroll_action::MotionControllFeedback feedback;
	feedback.part_id = part_id;
	feedback.actualCurrentState = msg->processedPart;
	feedback.currentSceneDuration = msg->actualCurrentSceneDuration;
	motionControllFeedbacks.push( feedback );

	currentSceneIdProcessed++;

	return SUCCESS;
}


CattyError MotionController::update_locomotionActionResultMsg( const support_msgs::ActionEndReporterMsg::ConstPtr & msg, const std::string & nodeName  ){
	if ( part_id != msg->part_id ){
		ROS_ERROR("PART_ID_NOT_MATCH(%s): Could not set the resultMsg of Action by support_msgs::ActionEndReporterMsg since PartID not match", nodeName.c_str());
		return PART_ID_NOT_MATCH;
	}

	while ( currentSceneIdProcessed < sequenceSize ){
		currentSceneIdProcessed++;
	}

	motionControllResult.part_id = part_id;
	motionControllResult.actualTotalDuration = ros::Time::now() - timeOfActionStart;
	motionControllResult.stateOfScenes = msg->stateOfScenes;

	return SUCCESS;
}


bool MotionController::set_locomotionActionFeedbackMsg( motioncontroll_action::MotionControllFeedback & feedback ) {
	if ( !motionControllFeedbacks.empty() ){
		feedback = motionControllFeedbacks.front();
		motionControllFeedbacks.pop();
		return true;
	}
	return false;
}


motioncontroll_action::MotionControllResult MotionController::get_locomotionActionResultMsg() const {
	return motionControllResult;
}


ros::Duration MotionController::get_expectedSceneDuration() const {
	return expectedSceneDuration;
}


CattyError MotionController::set_actionStartNotifier( support_msgs::ActionStartNotifierMsg & msg ) {
	if( msg.part_id == 0 ){
		msg.part_id = part_id;
		msg.sequenceSize = sequenceSize;
		msg.expectedSceneDuration = expectedSceneDuration;
		return SUCCESS;
	} else if ( part_id == msg.part_id ) {
		msg.sequenceSize = sequenceSize;
		msg.expectedSceneDuration = expectedSceneDuration;
		return SUCCESS;
	} else {
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
}

void MotionController::reset(){
	initialize_feedbackMsg();
	initialize_resultMsg();
	expectedStates.clear();
}


void MotionController::end_action(){
	inAction = false;
}

bool MotionController::isInAction() const { return inAction; }

bool MotionController::isValid() const { return valid; }
