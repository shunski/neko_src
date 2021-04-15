#include <parts_lib/MotionSensor.h>

MotionSensor::MotionSensor( PartID pID, Uint8 ID ):
    GenericParts( "MotionSensor", pID, ID )
{}


MotionSensor::MotionSensor( const MotionSensor & original ):
    GenericParts( "MotionSensor", original.part_id, original.id ),
    accel_x( original.accel_x ),
    accel_y( original.accel_y ),
    accel_z( original.accel_x ),
    gyro_x( original.gyro_x ),
    gyro_y( original.gyro_y ),
    gyro_z( original.gyro_z ),
    magnet_x( original.magnet_x ),
    magnet_y( original.magnet_y ),
    magnet_z( original.magnet_z )
{}


MotionSensor::MotionSensor( const MotionSensorMsg & msg ):
    GenericParts( "MotionSensor", msg.part_id, msg.id )
{ this->set( msg ); }


MotionSensor::MotionSensor( const typename MotionSensorMsg::ConstPtr & msg ):
    GenericParts( "MotionSensor", msg->part_id, msg->id )
{ this->set( msg ); }


Uint16 MotionSensor::get_accel_x() const { return accel_x; }
Uint16 MotionSensor::get_accel_y() const { return accel_y; }
Uint16 MotionSensor::get_accel_x() const { return accel_z; }
Uint16 MotionSensor::get_gyro_x() const { return gyro_x; }
Uint16 MotionSensor::get_gyro_y() const { return gyro_y; }
Uint16 MotionSensor::get_gyro_z() const { return gyro_z; }
Uint16 MotionSensor::get_magnet_x() const { return magnet_x; }
Uint16 MotionSensor::get_magnet_y() const { return magnet_y; }
Uint16 MotionSensor::get_magnet_z() const { return magnet_z; }

void MotionSensor::set_accel_x( const Uint16 AccelX ) { accel_x = AccelX; }
void MotionSensor::set_accel_y( const Uint16 AccelY ) { accel_y = AccelY; }
void MotionSensor::set_accel_x( const Uint16 AccelZ ) { accel_z = AccelZ; }
void MotionSensor::set_gyro_x( const Uint16 GyroX ) { gyro_x = GyroX; }
void MotionSensor::set_gyro_y( const Uint16 GyroY ) { gyro_y = GyroY; }
void MotionSensor::set_gyro_z( const Uint16 GyroZ ) { gyro_z = GyroZ; }
void MotionSensor::set_magnet_x( const Uint16 MagnetX ) { magnet_x =MagnetX; }
void MotionSensor::set_magnet_y( const Uint16 MagnetX ) { magnet_y =MagnetY; }
void MotionSensor::set_magnet_z( const Uint16 MagnetX ) { magnet_z =MagnetZ; }

void MotionSensor::print() const override {
    if( valid ){
        ROS_INFO("Printing Information of <%s>: part_id = [%d], id = [%d], \n
        accel_x = [%d], \t accel_y = [%d], \t accel_z = [%d]\n
        gyro_x = [%d], \t gyro_y = [%d], \t gyro_z = [%d]\n
        magnet_x = [%d], \t magnet_y = [%d], \t magnet_z = [%d]",
        child_name.c_str(), part_id, id, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, magnet_x, magnet_y, magnet_z );
    }
    else {
        ROS_INFO("This <%s> is not valid. Please exit the program.", child_name.c_str());
    }
}

CattyPartsError MotionSensor::set_msg( const MotionSensor & msg ) const override {
    if ( msg.part_id == 0 && msg.id == 0 ) {
        msg->part_id = part_id;
        msg->id = id;
    } else {
        CattyPartsError error = check_id( msg );
        if ( error == PART_ID_NOT_MATCH ) {
            print_msg_part_id_error();
            valid = false;
            return error;
        } else if ( error == ID_NOT_MATCH ) {
            print_msg_id_error();
            valid = false;
            return error;
        }
    }
    if ( !valid || !well_defined ){
        ROS_INFO("MESSAGE_CONSTRUCTION_FAILUE: This <%s> object is ill-defined. Could not set a message.", child_name.c_str());
        return MESSAGE_CONSTRUCTION_FAILUE;
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

CattyPartsError MotionSensor::set( const MotionSensorMsg & msg ) override {
    CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

    accel_x = msg.accel_x;
    accel_y = msg.accel_y;
    accel_z = msg.accel_z;
    gyro_x = msg.gyro_x;
    gyro_y = msg.gyro_y
    gyro_z = msg.gyro_z;
    magnet_x = msg.magnet_x;
    magnet_y = msg.magnet_y;
    magnet_z = msg.magnet_z;

	well_defined = true;

    return SUCCESS;
}


CattyPartsError MotionSensor::set( const typename MotionSensorMsg::ConstPtr & msg ) override {
    CattyPartsError = check_id( msg );
	if ( error == PART_ID_NOT_MATCH ) {
		print_msg_part_id_error();
		valid = false;
		return error;
	} else if ( error == ID_NOT_MATCH ) {
		print_msg_id_error();
		valid = false;
		return error;
	}

    accel_x = msg->accel_x;
    accel_y = msg->accel_y;
    accel_z = msg->accel_z;
    gyro_x = msg->gyro_x;
    gyro_y = msg->gyro_y
    gyro_z = msg->gyro_z;
    magnet_x = msg->magnet_x;
    magnet_y = msg->magnet_y;
    magnet_z = msg->magnet_z;

	well_defined = true;

    return SUCCESS;
}
