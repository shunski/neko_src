#include <support_lib/Utilities.h>

std::string get_description( ComponentId component_id ){
    if ( component_id == KondoServo ){
        return "KondoServo";
    } else if ( component_id == Motor ){
        return "Motor";
    } else {
        return "MotionSensor";
    };
}

std::string createRandomStringOfSize( int n ){
    std::string str;
    for( int i=0; i<n; i++ ){
        str.push_back( rand()%26 + 65 );
    }
    return str;
}

PartProperties::PartProperties( PartID ID, size_t KondoServoNum, size_t MotorNum, size_t MotionSensorNum ):
    id( ID ),
    kondoServoNum( KondoServoNum ),
    brushedMotorNum( MotorNum ),
    motionSensorNum( MotionSensorNum )
{}


PartProperties get_properties_by_id( PartID id ){
    switch ( id )
    {
        case( HEAD ):    return PartProperties( HEAD,  1, 0 , 0 );
        case( CHEST ):   return PartProperties( CHEST, 2, 0 , 0 );
        case( WAIST ):   return PartProperties( WAIST, 3, 1 , 0 );
        case( RFLEG ):   return PartProperties( RFLEG, 1, 1 , 2 );
        case( LFLEG ):   return PartProperties( LFLEG, 1, 1 , 2 );
        case( RHLEG ):   return PartProperties( RHLEG, 3, 1 , 2 );
        case( LHLEG ):   return PartProperties( LHLEG, 3, 1 , 2 );
    }
    return PartProperties( NONE, 0, 0, 0, 0);
}


std::string get_description( ObjectState state ){
    switch( state )
    {
        case( GENERAL ): return "GENERAL";
        case( COMMAND ): return "COMMAND";
        case( FEEDBACK ): return "FEEDBACK";
        default: return "NONE";
    }
}


bool isSucceeded( CattyError error ) { return !error; }

std::string get_description( CattyError error ) {
    switch( error )
    {
        case SUCCESS: return "SUCCESS";
		case WARNING: return "WARNING";
        case LOCOMOTION_ACTION_FAILURE: return "LOCOMOTION_ACTION_FAILURE";
		case MOTION_INIT_FAILURE: return "MOTION_INIT_FAILURE";
        case OBJECT_CONSTRUCTION_FAILURE: return "OBJECT_CONSTRUCTION_FAILURE";
        case MESSAGE_CONSTRUCTION_FAILURE: return "MESSAGE_CONSTRUCTION_FAILURE";
        case PART_ID_NOT_MATCH: return "PART_ID_NOT_MATCH";
		default: return "COMPONENT_ID_NOT_MATCH";
    }
}
