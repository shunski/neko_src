// FeedbackProcessor.cpp implements FeedbackProcessor class in Body.h
#include <body_lib/Body.h>
using namespace Body;

FeedbackProcessor::FeedbackProcessor( PartID ID ):
	part_id( ID ), inAction( false )
{}


CattyError FeedbackProcessor::start_action( const support_msgs::ActionStartNotifierMsg::ConstPtr & msg, std::string & NodeName ){
	if ( inAction ){
		ROS_INFO("LOCOMOTION_ACTION_ERROR: %s could not start action since another action is in already happning. ", NodeName.c_str());
		valid = false;
		return LOCOMOTION_ACTION_ERROR;
	}
	if ( msg->part_id != part_id ){
		valid = false;
		ROS_INFO("PART_ID_NOT_MATCH: %s could not start action since StartActionNotifierMsg id does not match. ", NodeName.c_str());
		return PART_ID_NOT_MATCH;
	}
	inAction = true;
	sequenceSize = msg->sequenceSize;
	reset();
}


bool FeedbackProcessor::add_pendingScenes( const teensy_msgs::CommandMsg::ConstPtr & msg )
{
	if ( msg->scene_id <= currentSceneIdReceived ){

		reset_processing();

		if ( msg->scene_id != 0) { // if some starting scenes are missed
			Uint16 numMissedHere = msg->scene_id - 1;
			numTotallyMissingScenes += numMissedHere;
		}
		pendingScenes.push( Part(msg) );

	} else if ( msg->scene_id > currentSceneIdReceived+1 ) {

		Uint16 numMissedHere = msg->scene_id - (currentSceneIdReceived+1);
		numTotallyMissingScenes += numMissedHere;
		pendingScenes.push( Part( msg ));

	} else { // msg->scene_id == currentSceneIdReceived+1 what we expect
		pendingScenes.push( Part( msg ));
	}

	currentSceneIdReceived = msg->scene_id;

	if ( currentSceneIdReceived == sequenceSize ) {
		ros::Duration(0.5).sleep();
		if ( currentSceneIdProcessed != currentSceneIdReceived )
			fillTheRest();
		inAction = false;
		return true;
	}

	return false;
}


void FeedbackProcessor::fillTheRest() {
	while( stateOfScenes.size() < sequenceSize )
		stateOfScenes.push_back( COMMAND );
}


Part FeedbackProcessor::process_Feedback( const teensy_msgs::FeedbackMsg::ConstPtr & msg )
{
	if( pendingScenes.front().get_scene_id() == msg->scene_id )
	{
		Part processedPart = pendingScenes.front();
		CatttyError error = processedPart.set( msg );
		if ( error!=SUCCESS )
		{
			valid = false;
		}
		pendingScenes.pop();
		currentSceneIdProcessed++;
		return processedPart;
	}

	else if ( pendingScenes.front().get_scene_id() > msg->scene_id )
	{
		currentSceneIdProcessed = msg->scene_id;
		numMissingExpectedScenes++;
		numTotallyMissingScenes--;
		return Part( msg );
	}


	else // pendingScenes.front().scene_id() < msg->scene_id
	{
		currentSceneIdProcessed = msg->scene_id;
		while ( pendingScenes.front().get_scene_id() != msg->scene_id ){
			pendingScenes.pop();
			numMissingActualScenes++;
		}
		Part processedPart = pendingScenes.front();
		CattyError error = processedPart.set(msg);
		if ( error!=SUCCESS )
		{
			valid = false;
		}
		pendingScenes.pop();
		return processedPart;
	}
}


void FeedbackProcessor::reset()
{
	pendingScenes = queue<Part>( sequenceSize, INVALID );
	numMissingActualScenes = 0;
	numMissingExpectedScenes = 0;
	numTotallyMissingScenes = 0;
}


bool FeedbackProcessor::isInAction() const { return inAction; }

bool FeedbackProcessor::isValid() const { return valid; }
