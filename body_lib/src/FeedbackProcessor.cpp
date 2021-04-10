// FeedbackProcessor.cpp implements FeedbackProcessor class in Body.h
#include <body_lib/Body.h>
using namespace Body;

FeedbackProcessor::FeedbackProcessor( PartID ID )
	previousState( ID ),
	currentState( ID )
{}

Part FeedbackProcessor::processFeedback( teesny_msgs::FeedbackMsg::ConstPtr & msg ) {
    currentState.set( msg );
}