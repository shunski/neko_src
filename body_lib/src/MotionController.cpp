// MotionController.cpp: implements MotionController class in Body.h
#include <body_lib/Body.h>
namespace Body{


MotionController::MotionController( PartID ID ):
	part_id( ID ),
	actualCurrentState( ID ),
	valid( true )
{}


CattyError MotionController::set_action( const motioncontroll_action::MotionControllGoal::ConstPtr & msg ) {
	if ( part_id != msg->part_id ){
        ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since message id does not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
    }

	if ( msg->sequenceSize != msg->partCommandSequence.size() ){
		ROS_INFO("ERROR: Could not set MotionController object from body_msgs::PartCommandMsg since the message is not well defined; sizes do not match.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
	}

	if ( inAction == true ) {
		ROS_INFO("ERROR: Could not set MotionController object to start since another action is in progress.");
		valid = false;
        return LOCOMOTION_ACTION_FAILURE;
	}
	inAction = true;

	initialize_feedbackMsg();
	initialize_resultMsg();
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
	if( expectedStates.size() != msg->sequenceSize ) return LOCOMOTION_ACTION_FAILURE;
	return SUCCESS;
}


// CattyError MotionController::init_part( init_service::PartInit::Request &,
// 										init_service::PartInit::Response & ){
//
// };

// CattyError MotionController::startMotioncontrollAction( motioncontroll_action::MotionControllGoal::ConstPtr & ) {
// 	if ( expectedStates.front() == actualCurrentState ) {
// 		ROS_INFO("The expected position too far from the current position and thus could not start action.");
// 		return LOCOMOTION_ACTION_FAILURE;
// 	}
// 	timeOfActionStart = ros::Time::now();
// }


CattyError MotionController::initialize_feedbackMsg(){
	motionControllFeedback = motioncontroll_action::MotionControllFeedback();
	motionControllFeedback.part_id = part_id;
}


CattyError MotionController::initialize_resultMsg(){
	motionControllResult = motioncontroll_action::MotionControllResult();
	motionControllResult.part_id = part_id;
}


void MotionController::proceed() {
    if (expectedCurrentState == expectedStates.begin()){ //first case
        ros::Time currentTime = ros::Time::now();
        actualCurrentSceneDuration.push( currentTime - timeOfActionStart );
        timeOfLastAction = currentTime;
    }
    else{
        ros::Time currentTime = ros::Time::now();
        actualCurrentSceneDuration.push( currentTime - timeOfLastAction );
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


CattyError MotionController::update_locomotionActionFeedbackMsg( const body_msgs::PartMsg::ConstPtr & msg, const std::string & nodeName ){
	if ( part_id != msg->part_id ){
		ROS_INFO("PART_ID_NOT_MATCH(%s): Could not set the feedbackMsg of Action by body_msgs::PartMsg since PartID not match", nodeName.c_str());
		return PART_ID_NOT_MATCH;
	}

	while ( currentSceneIdProcessed < msg->scene_id ){
		actualCurrentSceneDuration.pop();
		currentSceneIdProcessed++;
	}

	motionControllFeedback.part_id = part_id;
	motionControllFeedback.actualCurrentState = *msg;
	motionControllFeedback.currentSceneDuration = actualCurrentSceneDuration.front();

	actualCurrentSceneDuration.pop();
	currentSceneIdProcessed++;

	return SUCCESS;
}


CattyError MotionController::update_locomotionActionResultMsg( const support_msgs::ActionEndReporterMsg::ConstPtr & msg, const std::string & nodeName  ){
	if ( part_id != msg->part_id ){
		ROS_INFO("PART_ID_NOT_MATCH(%s): Could not set the resultMsg of Action by support_msgs::ActionEndReporterMsg since PartID not match", nodeName.c_str());
		return PART_ID_NOT_MATCH;
	}

	while ( currentSceneIdProcessed < sequenceSize ){
		actualCurrentSceneDuration.pop();
		currentSceneIdProcessed++;
	}

	motionControllResult.part_id = part_id;
	motionControllResult.actualTotalDuration = ros::Time::now() - timeOfActionStart;
	motionControllResult.stateOfScenes = msg->stateOfScenes;

	return SUCCESS;
}


motioncontroll_action::MotionControllFeedback MotionController::get_locomotionActionFeedbackMsg()const {
	return motionControllFeedback;
}


motioncontroll_action::MotionControllResult MotionController::get_locomotionActionResultMsg() const {
	return motionControllResult;
}


ros::Duration MotionController::get_actualCurrentSceneDuration() const {
	return actualCurrentSceneDuration.front();
}


ros::Duration MotionController::get_expectedSceneDuration() const {
	return expectedSceneDuration;
}


CattyError MotionController::set_actionStartNotifier( support_msgs::ActionStartNotifierMsg & msg ) {
	if( msg.part_id == 0 ){
		msg.part_id = part_id;
		msg.sequenceSize = sequenceSize;
		return SUCCESS;
	} else if ( part_id == msg.part_id ) {
		msg.sequenceSize = sequenceSize;
		return SUCCESS;
	} else {
		return MESSAGE_CONSTRUCTION_FAILURE;
	}
}


void MotionController::add_processedFeedbackPending( const body_msgs::PartMsg::ConstPtr & msg ){
	body_msgs::PartMsg msgCopy = *msg;
	processedFeedbackPending.push( msgCopy );
}


void MotionController::end_action(){
	inAction = false;
}

bool MotionController::isInAction() const { return inAction; }

bool MotionController::isValid() const { return valid; }


} // closing namespace
