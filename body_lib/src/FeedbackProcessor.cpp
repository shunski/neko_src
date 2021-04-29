// FeedbackProcessor.cpp implements FeedbackProcessor class in Body.h
#include <body_lib/Body.h>
using namespace body;

FeedbackProcessor::FeedbackProcessor( PartID ID ):
	part_id( ID ), inAction( false )
{}


CattyError FeedbackProcessor::start_action( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg, const std::string & NodeName ){
	if ( inAction ) {
		ROS_ERROR("LOCOMOTION_ACTION_ERROR: %s could not start action since another action is already happning. ", NodeName.c_str());
		valid = false;
		return LOCOMOTION_ACTION_FAILURE;
	}
	if ( msg->part_id != part_id ) {
		valid = false;
		ROS_ERROR("PART_ID_NOT_MATCH: %s could not start action since StartActionNotifierMsg id does not match. ", NodeName.c_str());
		return PART_ID_NOT_MATCH;
	}
	inAction = true;
	sequenceSize = msg->sequenceSize;
	expectedSceneDuration = msg->expectedSceneDuration;
	reset();
	return SUCCESS;
}


bool FeedbackProcessor::add_pendingScenes( const teensy_msgs::CommandMsg::ConstPtr & msg )
{
	if (msg->scene_id == 0){ // initial scene of an action

		pendingScenes.push( Part( msg ));

	} else if ( msg->scene_id <= currentSceneIdReceived ){

		ROS_ERROR("Some exeption occured while adding command to the list");
		reset();
		if ( msg->scene_id != 0) { // if some starting scenes are missed
			Uint16 numMissedHere = msg->scene_id - 1;
			numTotallyMissingScenes += numMissedHere;
		}
		pendingScenes.push( Part(msg) );

	} else if ( msg->scene_id > currentSceneIdReceived+1 ) {

		ROS_ERROR("Some exeption occured while adding command to the list");
		Uint16 numMissedHere = msg->scene_id - ( currentSceneIdReceived+1 );
		numTotallyMissingScenes += numMissedHere;
		pendingScenes.push( Part( msg ));

	} else { // msg->scene_id == currentSceneIdReceived+1. This is what we expect

		pendingScenes.push( Part( msg ));

	}

	ROS_INFO("The scene of scene_id [%d] and of part_id[%d] is added to the pending scenes", msg->scene_id ,msg->part_id );
	currentSceneIdReceived = msg->scene_id;

	if ( currentSceneIdReceived == sequenceSize-1 ) return true; // returning true to end action
	return false;
}


void FeedbackProcessor::fillTheRest() {
	while( stateOfScenes.size() < sequenceSize )
		stateOfScenes.push_back( COMMAND );
}


void FeedbackProcessor::process_Feedback( const teensy_msgs::FeedbackMsg::ConstPtr & msg, body_msgs::ProcessedFeedbackMsg& processedFeedback )
{
	if( pendingScenes.size() == 0 ){
		ROS_ERROR("A feedback from teensy is received though there is no pending scene.");
		return;
	}
	Part processedPart( part_id );
	if( pendingScenes.front().get_scene_id() == msg->scene_id )
	{
		processedPart = pendingScenes.front();
		CattyError error = processedPart.set( msg );
		if ( error != SUCCESS )
		{
			valid = false;
		}
		pendingScenes.pop();
		currentSceneIdProcessed = msg->scene_id;
	}
	else if ( pendingScenes.front().get_scene_id() > msg->scene_id ) // Some feedbacks from the teensy were missed
	{
		currentSceneIdProcessed = msg->scene_id;
		while ( pendingScenes.front().get_scene_id() != msg->scene_id ){
			pendingScenes.pop();
			numMissingActualScenes++;
		}
		processedPart = pendingScenes.front();
		CattyError error = processedPart.set(msg);
		if ( error!=SUCCESS )
		{
			valid = false;
		}
		pendingScenes.pop();
	}
	else // pendingScenes.front().scene_id() < msg->scene_id : some commands from the motion controller were missed
	{
		currentSceneIdProcessed = msg->scene_id;
		numMissingExpectedScenes++;
		numTotallyMissingScenes--;
		processedPart = Part( msg );
	}

	body_msgs::PartMsg processedPartMsg;
	processedPart.set_PartMsg( processedPartMsg );
	processedFeedback.processedPart = processedPartMsg;
	processedFeedback.actualCurrentSceneDuration = msg->actualCurrentSceneDuration;

	return;
}


void FeedbackProcessor::reset()
{
	while( !pendingScenes.empty()) pendingScenes.pop();
	numMissingActualScenes = 0;
	numMissingExpectedScenes = 0;
	numTotallyMissingScenes = 0;
	currentSceneIdReceived = 0;
	currentSceneIdProcessed = 0;
	stateOfScenes.clear();
}


CattyError FeedbackProcessor::set_ActionEndReporterMsg( support_msgs::ActionEndReporterMsg & msg ){
	msg.part_id = part_id;

	msg.stateOfScenes.clear();
	msg.stateOfScenes.reserve( sequenceSize );
	for ( std::vector<ObjectState>::iterator it = stateOfScenes.begin(); it!=stateOfScenes.end(); ++it )
		msg.stateOfScenes.push_back( *it );
	return SUCCESS;
}

bool FeedbackProcessor::isInAction() const { return inAction; }

Uint8 FeedbackProcessor::get_scenesLeft() const{ return sequenceSize - currentSceneIdProcessed; }

ros::Duration FeedbackProcessor::get_expectedSceneDuration() const { return expectedSceneDuration; }

bool FeedbackProcessor::isValid() const { return valid; }

void FeedbackProcessor::end_action(){
	inAction = false;
	reset();
}
