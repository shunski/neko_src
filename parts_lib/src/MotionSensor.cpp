#include <parts_lib/MotionSensor.h>

MotionSensor::MotionSensor( PartID pID, Uint8 ID ):
    GenericParts( "MotionSensor", pID, ID )
{}


MotionSensor::MotionSensor( const MotionSensor & original ):
    GenericParts( "MotionSensor", PartID( original.part_id ), original.id ),
    accel_x( original.accel_x ),
    accel_y( original.accel_y ),
    accel_z( original.accel_x ),
    gyro_x( original.gyro_x ),
    gyro_y( original.gyro_y ),
    gyro_z( original.gyro_z ),
    magnet_x( original.magnet_x ),
    magnet_y( original.magnet_y ),
    magnet_z( original.magnet_z )
{
    state = original.state;
    valid = original.valid;
    well_defined = original.well_defined;
}


MotionSensor::MotionSensor( const MotionSensorMsg & msg ):
    GenericParts( "MotionSensor", PartID( msg.part_id ), msg.id )
{ this->set( msg ); }


MotionSensor::MotionSensor( const typename MotionSensorMsg::ConstPtr & msg ):
    GenericParts( "MotionSensor", PartID( msg->part_id ), msg->id )
{ this->set( msg ); }


Uint16 MotionSensor::get_accel_x() const { return accel_x; }
Uint16 MotionSensor::get_accel_y() const { return accel_y; }
Uint16 MotionSensor::get_accel_z() const { return accel_z; }
Uint16 MotionSensor::get_gyro_x() const { return gyro_x; }
Uint16 MotionSensor::get_gyro_y() const { return gyro_y; }
Uint16 MotionSensor::get_gyro_z() const { return gyro_z; }
Uint16 MotionSensor::get_magnet_x() const { return magnet_x; }
Uint16 MotionSensor::get_magnet_y() const { return magnet_y; }
Uint16 MotionSensor::get_magnet_z() const { return magnet_z; }


void MotionSensor::set_accel_x( const Uint16 AccelX ) { accel_x = AccelX; }
void MotionSensor::set_accel_y( const Uint16 AccelY ) { accel_y = AccelY; }
void MotionSensor::set_accel_z( const Uint16 AccelZ ) { accel_z = AccelZ; }
void MotionSensor::set_gyro_x( const Uint16 GyroX ) { gyro_x = GyroX; }
void MotionSensor::set_gyro_y( const Uint16 GyroY ) { gyro_y = GyroY; }
void MotionSensor::set_gyro_z( const Uint16 GyroZ ) { gyro_z = GyroZ; }
void MotionSensor::set_magnet_x( const Uint16 MagnetX ) { magnet_x =MagnetX; }
void MotionSensor::set_magnet_y( const Uint16 MagnetY ) { magnet_y =MagnetY; }
void MotionSensor::set_magnet_z( const Uint16 MagnetZ ) { magnet_z =MagnetZ; }


void MotionSensor::operator=( const MotionSensor & original ){
    if( part_id != original.part_id || id != original.id ){
        ROS_INFO("Invalid use of assignment operator for two MotionSensor objects. Operation failed.");
        return;
    }

    valid = original.valid;
    well_defined = original.well_defined;
    state = original.state;

    accel_x = original.accel_x;
    accel_y = original.accel_y;
    accel_z = original.accel_z;
    gyro_x = original.gyro_x;
    gyro_y = original.gyro_y;
    gyro_z = original.gyro_z;
    magnet_x = original.magnet_x;
    magnet_y = original.magnet_y;
    magnet_z = original.magnet_z;

}


void MotionSensor::print() const {
    if( valid ){
        ROS_ERROR("Printing Information of <%s>: part_id = [%d], id = [%d], \naccel_x = [%d], \t accel_y = [%d], \t accel_z = [%d]\ngyro_x = [%d], \t gyro_y = [%d], \t gyro_z = [%d]\nmagnet_x = [%d], \t magnet_y = [%d], \t magnet_z = [%d]", child_name.c_str(), part_id, id, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, magnet_x, magnet_y, magnet_z );
    }
    else {
        ROS_ERROR("This <%s> is not valid. Please exit the program.", child_name.c_str());
    }
}


CattyError MotionSensor::set_msg( MotionSensorMsg & msg ) const {
    if ( msg.part_id == 0 && msg.id == 0 ) {
        msg.part_id = part_id;
        msg.id = id;
    } else {
        CattyError error = check_msg_id( msg );
        if ( error == PART_ID_NOT_MATCH ) {
            print_msg_part_id_error( msg );
            return error;
        } else if ( error == ID_NOT_MATCH ) {
            print_msg_id_error( msg );
            return error;
        }
    }
    if ( !valid || !well_defined ){
        ROS_ERROR("MESSAGE_CONSTRUCTION_FAILURE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
        return MESSAGE_CONSTRUCTION_FAILURE;
    }

    msg.accel_x = accel_x;
    msg.accel_y = accel_y;
    msg.accel_z = accel_z;
    msg.gyro_x = gyro_x;
    msg.gyro_y = gyro_y;
    msg.gyro_z = gyro_z;
    msg.magnet_x = magnet_x;
    msg.magnet_y = magnet_y;
    msg.magnet_z = magnet_z;

    return SUCCESS;
}


CattyError MotionSensor::set( const MotionSensorMsg & msg ) {
    CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    accel_x = msg.accel_x;
    accel_y = msg.accel_y;
    accel_z = msg.accel_z;
    gyro_x = msg.gyro_x;
    gyro_y = msg.gyro_y;
    gyro_z = msg.gyro_z;
    magnet_x = msg.magnet_x;
    magnet_y = msg.magnet_y;
    magnet_z = msg.magnet_z;

	well_defined = true;

    return SUCCESS;
}


CattyError MotionSensor::set( const typename MotionSensorMsg::ConstPtr & msg ) {
    CattyError error = check_msg_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error( msg );
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error( msg );
		valid = false;
		return error;
	}

    accel_x = msg->accel_x;
    accel_y = msg->accel_y;
    accel_z = msg->accel_z;
    gyro_x = msg->gyro_x;
    gyro_y = msg->gyro_y;
    gyro_z = msg->gyro_z;
    magnet_x = msg->magnet_x;
    magnet_y = msg->magnet_y;
    magnet_z = msg->magnet_z;

	well_defined = true;

    return SUCCESS;
}
