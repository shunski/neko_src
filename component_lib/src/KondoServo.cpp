// KondoServo.cpp: implements KondoServo.h
#include <component_lib/component.h>

// Initialization for FeedbackProcessor
KondoServo::KondoServo( PartID part_id, ComponentId component_id, IdNumber id_number, const Command& initial_position ):
    Actuator( part_id, component_id, id_number, COMMAND )
    command( command )
{}

// Initialization for FeedbackProcessor
KondoServo::KondoServo( PartId part_id, ComponentId component_id, IdNumber id_number ):
    Actuator( part_id, component_id, id_number )
{}

// Initialization for learning Node
KondoServo::KondoServo( PartId part_id, ComponentId component_id, IdNumber id_number, TransferableObjectState state, Bounds bounds, ParametersForSpaceGeneration parameters_for_space_generation ):
    Actuator( part_id, component_id, id_number, state ),
    bounds( bounds ),
    parameters_for_space_generation( parameters_for_space_generation )
{}

// Explicit copy constructor
KondoServo::KondoServo( const KondoServo & original ):
	Actuator( PartId(original.part_id), original.component_id, original.id_number, valid.valid ),
	command(command),
    feedback(feedback)
{}

// implementations of struct defined in the class
void KondoServo::Command::operator=( const component_lib::KondoServoCommandMsg::ConstPtr& msg ){
    msg.ideal_position = ideal_position;
    msg.temp_limit = temp_limit;
    msg.current_limit = current_limit;
    msg.speed = speed;
    msg.stretch = stretch;
    msg.free = free;
}


void KondoServo::Command::fill( component_lib::KondoServoCommandMsg& msg ){
    ideal_position = msg.ideal_position;
    temp_limit = msg.temp_limit;
    current_limit = msg.current_limit;
    speed = msg.speed;
    stretch = msg.stretch;
    free = msg.free;
}


void KondoServo::Feedback::operator=( const component_lib::KondoServoFeedbackMsg::ConstPtr& msg ){
    msg.actual_position = actual_position;
    msg.temp = temp;
    msg.current = current;
    msg.is_freed = is_freed;
}


void KondoServo::Feedback::fill( component_lib::KondoServoFeedbackMsg& msg ){
    actual_position = msg.actual_position;
    temp = msg.temp;
    current = msg.current;
    is_freed = msg.is_freed;
}


void KondoServo::operator=( const KondoServo & original)
{
    CattyError error = this->check_id( original );
    if( error ){
        this->print_id_error( error, original );
        valid = false;
        return error;
    }

    command = original.command;
    feedback = original.feedback;

    return;
}


const Command& KondoServo::get_command_parameters(){ return command; }
const Feedback& KondoServo::get_feedback_parameters(){ return feedback; }
const Bounds& KondoServo::get_bounds(){ return bounds; }
const ParametersForSpaceGeneration& KondoServo::get_parameters_for_space_generation(){ return parameters_for_space_generation; }


void KondoServo::print() const
{}


void KondoServo::fill_command_msg( component_lib::KondoServoCommandMsg & msg ) const {
    command.fill( msg );
}


void KondoServo::fill_feedback_msg( component_lib::KondoServoFeedbackMsg & msg ) const {
    feedback.fill( msg );
}


void KondoServo::set( const component_lib::KondoServoCommandMsg::ConstPtr & msg ){
    command = msg;
}

void KondoServo::set( const component_lib::KondoServoFeedbackMsg::ConstPtr & ){
    feedback = msg;
}
