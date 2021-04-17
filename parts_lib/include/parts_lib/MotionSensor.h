#ifndef MOTIONSENSOR_H
#define MOTIONSENSOR_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_lib/GenericParts.h>
#include <parts_msgs/MotionSensorMsg.h>
#include <parts_msgs/MotionSensorCommandMsg.h>   // a dummy
#include <parts_msgs/MotionSensorFeedbackMsg.h>  // a dummy

typedef parts_msgs::MotionSensorMsg MotionSensorMsg;
typedef parts_msgs::MotionSensorCommandMsg MotionSensorCommandMsg;
typedef parts_msgs::MotionSensorFeedbackMsg MotionSensorFeedbackMsg;

class MotionSensor : public GenericParts<MotionSensorMsg, MotionSensorCommandMsg, MotionSensorFeedbackMsg> // the last two classes are dummies since no CommandMsg or FeedbackMsg required for this class
{
    private:
        Uint16 accel_x;
        Uint16 accel_y;
        Uint16 accel_z;
        Uint16 gyro_x;
        Uint16 gyro_y;
        Uint16 gyro_z;
        Uint16 magnet_x;
        Uint16 magnet_y;
        Uint16 magnet_z;

    public:
        MotionSensor( PartID, Uint8 ID );
        MotionSensor( const MotionSensor & );

        MotionSensor( const MotionSensorMsg & );
        MotionSensor( const typename MotionSensorMsg::ConstPtr & );

        Uint16 get_accel_x() const ;
        Uint16 get_accel_y() const ;
        Uint16 get_accel_z() const ;
        Uint16 get_gyro_x() const ;
        Uint16 get_gyro_y() const ;
        Uint16 get_gyro_z() const ;
        Uint16 get_magnet_x() const ;
        Uint16 get_magnet_y() const ;
        Uint16 get_magnet_z() const ;

        void set_accel_x( const Uint16 AccelX );
        void set_accel_y( const Uint16 AccelY );
        void set_accel_z( const Uint16 AccelZ );
        void set_gyro_x( const Uint16 GyroX );
        void set_gyro_y( const Uint16 GyroY );
        void set_gyro_z( const Uint16 GyroZ );
        void set_magnet_x( const Uint16 MagnetX );
        void set_magnet_y( const Uint16 MagnetY );
        void set_magnet_z( const Uint16 MagnetZ );

        void print() const override ;

        CattyError set_msg( MotionSensorMsg & ) const override;

        CattyError set( const MotionSensorMsg & ) override ;
        CattyError set( const typename MotionSensorMsg::ConstPtr & ) override ;
};

#endif
