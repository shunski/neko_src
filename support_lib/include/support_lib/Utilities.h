#ifndef UTILITIES_H
#define UTILITIES_H

#define default_queue_size 100;

#include <vector>
#include <string>


// Useful type definitions
typedef unsigned char Uint8;
typedef unsigned short Uint16;

typedef std::vector<Uint8> Uint8Sequence;
typedef std::vector<Uint16> Uint16Sequence;


// Part Definitions
enum PartID { HEAD, CHEST, WAIST, RFLEG, LFLEG, RHLEG, LHLEG };

struct PartProperties {
    PartID id;
    size_t kondoServoNum;
    size_t brushedMotorNum;
    size_t brushlessMotorNum;
    size_t gyrosensorNum;
    PartProperties( PartID ID, size_t KondoServoNum, size_t BrushedMotorNum, size_t BrushlessMotorNum, size_t GyrosensorNum ):
        id( ID ),
        kondoServoNum( KondoServoNum ),
        brushedMotorNum( BrushedMotorNum ),
        brushlessMotorNum( BrushlessMotorNum ),
        gyrosensorNum( GyrosensorNum )
    {}
    PartProperties();
}

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

// Error Handling
enum CattyError
{
    SUCCESS=0,
    LOCOMOTION_ACTION_ERROR,
    INIT_ERROR,
    CONSTRUCTOR_ERROR
};

bool isSucceeded( CattyError error ) { return !error; }

std::string get_catty_error_description( CattyError error ) {
    switch( error )
    {
        case SUCCESS: return "SUCCESS: ";
        case LOCOMOTION_ACTION_ERROR: return "LOCOMOTION_ACTION_ERROR: ";
        case CONSTRUCTOR_ERROR: return "CONSTRUCTOR_ERROR: ";
        case INIT_ERROR: return "INIT_ERROR: ";
    }
}


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
