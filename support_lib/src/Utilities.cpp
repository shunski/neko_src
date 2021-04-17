#include <support_lib/Utilities.h>

PartProperties::PartProperties( PartID ID, size_t KondoServoNum, size_t BrushedMotorNum, size_t BrushlessMotorNum, size_t MotionSensorNum ):
    id( ID ),
    kondoServoNum( KondoServoNum ),
    brushedMotorNum( BrushedMotorNum ),
    brushlessMotorNum( BrushlessMotorNum ),
    motionSensorNum( MotionSensorNum )
{}


PartProperties get_properties_by_id( PartID id ){
    PartProperties pp;
    switch ( id )
    {
        case( HEAD ):    return PartProperties( HEAD,  1, 0, 0 , 0 );
        case( CHEST ):   return PartProperties( CHEST, 2, 0, 0 , 0 );
        case( WAIST ):   return PartProperties( WAIST, 3, 0, 1 , 0 );
        case( RFLEG ):   return PartProperties( RFLEG, 1, 1, 0 , 2 );
        case( LFLEG ):   return PartProperties( LFLEG, 1, 1, 0 , 2 );
        case( RHLEG ):   return PartProperties( RHLEG, 3, 1, 0 , 2 );
        case( LHLEG ):   return PartProperties( LHLEG, 3, 1, 0 , 2 );
    }
    return PartProperties();
}

bool isSucceeded( CattyError error ) { return !error; }

std::string get_catty_error_description( CattyError error ) {
    switch( error )
    {
        case SUCCESS: return "SUCCESS";
		case WARNING: return "WARNING";
        case LOCOMOTION_ACTION_FAILUE: return "LOCOMOTION_ACTION_FAILUE";
		case MOTION_INIT_FAILUE: return "MOTION_INIT_FAILUE";
        case OBJECT_CONSTRUCTION_FAILUE: return "OBJECT_CONSTRUCTION_FAILUE";
        case MESSAGE_CONSTRUCTION_FAILUE: return "MESSAGE_CONSTRUCTION_FAILUE";
        case PART_ID_NOT_MATCH: return "PART_ID_NOT_MATCH";
		case ID_NOT_MATCH: return "ID_NOT_MATCH";
    }
}

