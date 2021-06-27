#ifndef UTILITIES_H
#define UTILITIES_H

#define default_queue_size 10

#include <vector>
#include <string>

typedef std::vector<uint8_t> Uint8Sequence;
typedef std::vector<uint16_t> Uint16Sequence;

// Components definitions
// ComponentKind indicates the groups of comoponent, made for traits
enum ComponentKind { COMPUTER, BATTERY, MICRO_CONTROLLER, ACTUATOR, SENSOR };
// ComponentId indicates each of specific components
enum ComponentId { KONDO_SERVO, MOTOR, MOTION_SENSOR, CURRENT_SENSOR, VOLTAGE_SENSOR,
                    11_1_LIPO, 7.6_LIFE, RASPI4B, JETSON_NANO, };
std::string get_description( ComponentId );

// Error Handling
enum CattyError
{
    SUCCESS=0,
	WARNING,
    LOCOMOTION_ACTION_FAILURE,
    MOTION_INIT_FAILURE,
	OBJECT_CONSTRUCTION_FAILURE,
	MESSAGE_CONSTRUCTION_FAILURE,
	PART_ID_NOT_MATCH,
	COMPONENT_ID_NOT_MATCH
};

bool is_succeeded( CattyError error );
std::string get_description( CattyError error );


// Useful functions
std::string create_random_string_of_size( int );

// Part Definitions
enum PartId { INVALID=0, HEAD, CHEST, WAIST, RFLEG, LFLEG, RHLEG, LHLEG };
enum TransferableObjectState { NONE=0, GENERAL, COMMAND, FEEDBACK };

struct PartProperties {
    PartID part_id;
    size_t kondo_servo_num;
    size_t motor_num;
    size_t motion_sensor_num;
    PartProperties( PartId, size_t kondo_servo_num, size_t brushless_motor_num, size_t motion_sensor_num );
};

PartProperties get_properties_by_id( PartId );
std::string get_description( TransferableObjectState );



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
