// FeedbackProcessor.cpp implements FeedbackProcessor class in Body.h
#include <jcatty_body_lib/Body.h>
using namespace Body;

FeedbackProcessor::FeedbackProcessor( jcatty_teensy_msgs::InfoMsg::ConstPtr & msg ) { this->set( msg ); }

void FeedbackProcessor::set( jcatty_teensy_msgs::InfoMsg::ConstPtr & msg ) { currentState.set( msg ); }

Part FeedbackProcessor::processFeedback( InfoMsg::ConstPtr & msg ) {
    previousState = currentState;
    currentState.set( msg );
}
