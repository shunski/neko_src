#include <support_lib/Utilities.h>

const std::string get_description( ComponentKind component_kind ){
    switch ( component_kind ) {
        case COMPUTER: return "Computer";
        case BATTERY: return "Battery";
        case MICRO_CONTROLLER: return "MicroController";
        case ACTUATOR: return "Actuator";
        case SENSOR: return "Sensor";
        default:
            ROS_ERROR("Not a valid component_id. Could not return the description.");
            return "";
    }
}

const std::string get_description( ComponentId component_id ){
    switch ( component_id ) {
        case KONDO_SERVO: return "KondoServo";
        case MOTOR: return "Motor";
        case MOTION_SENSOR: return "MotionSensor";
        case CURRENT_SENSOR: return "CurrntSensor";
        case VOLTAGE_SENSOR: return "VoltageSensor";
        case BATTERY: return "Battery";
        case DC_POWER: return "DcPower";
        case COMPUTER: return "Computer";
        case MICRO_CONTROLLER: return "MicroController";
        default:
            ROS_ERROR("Not a valid component_id. Could not return the description.");
            return "";
    };
}

const std::string createRandomStringOfSize( int n ){
    std::string str;
    for( int i=0; i<n; i++ ){
        str.push_back( rand()%26 + 65 );
    }
    return str;
}

PartProperties::PartProperties(
        PartId partId,
        size_t kondoServoNum,
        size_t motorNum,
        size_t motionSensorNum,
        size_t currentSensorNum,
        size_t batteryNum,
        size_t dcPowerNum,
        size_t computerNum,
        size_t microControllerNum
    ):
        part_id( partId ),
        kondo_servo_num( kondoServoNum ),
        motor_num( motorNum ),
        motion_sensor_num( motionSensorNum ),
        current_sensor_num( currentSensorNum ),
        battery_num( batteryNum ),
        dc_power_num( dcPowerNum ),
        computer_num( computerNum ),
        micro_controller_num( microControllerNum )
{}

const std::string get_description( PartId partId ){
    switch( partId ){
        case( HEAD ): return "HEAD";
        case( CHEST ): return "CHEST";
        case( BELLY ): return "BELLY";
        case( WAIST ): return "WAIST";
        case( RFLEG ): return "RFLEG";
        case( LFLEG ): return "LFLEG";
        case( RHLEG ): return "RHLEG";
        case( LHLEG ): return "LHLEG";
        default: return "INVALID";
    }
}

const std::string get_description( TransferableObjectState state ){
    switch( state )
    {
        case( GENERAL ): return "GENERAL";
        case( COMMAND ): return "COMMAND";
        case( FEEDBACK ): return "FEEDBACK";
        case( EXPECTED ): return "EXPECTED";
        default: return "NONE";
    }
}


bool isSucceeded( CattyError error ) { return !error; }

const std::string get_description( CattyError error ) {
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
