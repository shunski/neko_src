#ifndef GYROSENSOR_H
#define GYROSENSOR_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_msgs/GyroSensorMsg.h>

class GyroSensor
{
    private:
        const Uint8 id;
        const PartID part_id;
    public:
        GyroSensor( Uint8, PartID );
};

#endif
