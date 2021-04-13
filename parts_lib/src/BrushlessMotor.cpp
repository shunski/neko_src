#include <parts_lib/BrushlessMotor.h>

BrushlessMotor::BrushlessMotor( Uint8 ID, PartID partID ):
	id( ID ),
	part_id( partID )
{}

BrushlessMotor::BrushlessMotor( BrushlessCommandMsg & msg ):
	id( msg->id ),
	part_id( msg->id )
{
	this->set( msg );
}


BrushlessMotor::BrushlessMotor( BrushlessFeedbackMsg & msg ):
	id( msg->id ),
	part_id( msg->id )
{
	this->set( msg );
	valid = true;
}


BrushlessMotor::BrushlessMotor( typename BrushlessFeedbackMsg & );
BrushlessMotor::check_msg_id( msg ) const {
	part_id != part;
}


BrushlessMotor::Uint16 get_id() const ;
BrushlessMotor::Uint16 get_part_id() const ;
BrushlessMotor::Int16 get_voltage() const ;
BrushlessMotor::Uint16 get_position() const ;
BrushlessMotor::Int16 get_speed() const ;
BrushlessMotor::Uint16 get_current() const ;

BrushlessMotor::CattyPartsError set_voltage( Uint16 );
BrushlessMotor::CattyPartsError set_position( Uint16 );
BrushlessMotor::CattyPartsError set_speed( Uint16 );
BrushlessMotor::CattyPartsError set_current( Uint16 );
BrushlessMotor::CattyPartsError set_msg();

BrushlessMotor::void print();

BrushlessMotor::ServoCommandMsg get_CommandMsg() const;
BrushlessMotor::CattyPartsError set_msg( BrushlessMsg & ) const; 
BrushlessMotor::CattyPartsError set_CommandMsg( BrushlessCommandMsg & ) const;
BrushlessMotor::CattyPartsError set( typename ServoFeedbackMsg::ConstPtr & );
BrushlessMotor::CattyPartsError set( BrushlessFeedbackMsg & );
BrushlessMotor::CattyPartsError set( BrushlessCommandMsg & );
BrushlessMotor::CattyPartsError set( BrushlessMsg & ); 
BrushlessMotor::CattyPartsError set( typename BrushlessMsg::ConstPtr & );

CattyParsError BrushlessMottor::set_CommandMsg( BrushlessCommadMsg & ) const {
    CattyPartError error = check_id( msg );
    if ( error == PART_ID_NOT_MATCH || error == ID_NOT_MATCH ) return error;

    if ( !( state == COMMAND || state == OBJECT )) {
        ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <BrushlessMotor> object is not suitable for setting the command message.");
        return MESSAGE_CONSTRUCTION_FAILUE;
    }
    if ( !valid || !well_defined ){
        ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <BrushlessMotor> object is ill-defined. Could not set a command message.");
        return MESSAGE_CONSTRUCTION_FAILUE;
    }

    command_data.set( msg );

    return SUCCESS;
}
