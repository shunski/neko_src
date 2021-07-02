#ifndef UTILITIES_H
#define UTILITIES_H

#define default_queue_size 10

#include <vector>
#include <string>
#include <ros/ros.h>

typedef std::vector<uint8_t> Uint8Sequence;
typedef std::vector<uint16_t> Uint16Sequence;

// Components definitions
// ComponentKind indicates the groups of comoponent, made for traits
enum ComponentKind { INVALID_COMPONENT_KIND, POWER_SUPPLY, CIRCUITS, ACTUATOR, SENSOR };
const std::string get_description( ComponentKind );

// ComponentId indicates each of specific components
enum ComponentId { INVALID_COMPONENT_ID, KONDO_SERVO, MOTOR, MOTION_SENSOR, CURRENT_SENSOR, VOLTAGE_SENSOR,
                    BATTERY, DC_POWER, COMPUTER, MICRO_CONTROLLER };
const std::string get_description( ComponentId );

// Error Handling
enum CattyError
{
    SUCCESS=0,
	WARNING,
    LOCOMOTION_ACTION_FAILURE,
    MOTION_INIT_FAILURE,
	OBJECT_CONSTRUCTION_FAILURE,
	MESSAGE_CONSTRUCTION_FAILURE,
    OBJECT_STATE_NOT_MATCH,
	PART_ID_NOT_MATCH,
	COMPONENT_ID_NOT_MATCH
};

bool is_succeeded( CattyError error );
const std::string get_description( CattyError error );


// Useful functions
const std::string create_random_string_of_size( int );

// Part Definitions
enum PartId { INVALID_PART_ID=0, HEAD, CHEST, BELLY, WAIST, RFLEG, LFLEG, RHLEG, LHLEG };
enum TransferableObjectState { NONE=0, GENERAL, COMMAND, FEEDBACK, EXPECTED };

struct PartProperties{
    PartId part_id;
    size_t kondo_servo_num;
    size_t motor_num;
    size_t motion_sensor_num;
    size_t current_sensor_num;
    size_t battery_num;
    size_t dc_power_num;
    size_t computer_num;
    size_t micro_controller_num;
    PartProperties(
        PartId,
        size_t kondoServoNum,
        size_t motorNum,
        size_t motionSensorNum,
        size_t currentSensorNum,
        size_t batteryNum,
        size_t dcPowerNum,
        size_t computerNum,
        size_t microControllerNum
    );
};

const std::string get_description( PartId );
const std::string get_description( TransferableObjectState );

// Additional ID definitions: Each component is identified by PartId, ComponentId and IdNumber
typedef uint8_t IdNumber;

struct Point
{
    double x;
    double y;
    Point( double X, double Y ) : x(X), y(Y) {}
};

class Plot
{
    private:
        std::vector<Point> points;      // stored in such a way that the Point.x is an increasing order
        float x_max;
        float y_max;
        float y_min;
    public:
        Plot() : x_max(0), y_max(0), y_min(0) {}
        std::vector<Point> get_points(){ return points; }
        float get_xMax() const { return x_max; }
        float get_yMax() const { return y_max; }
        float get_yMin() const { return y_min; }
        bool add_point( Point p );

};

#endif
