#ifndef UTILITIES_H
#define UTILITIES_H

#define default_queue_size 100;

#include <vector>

typedef unsigned char Uint8;
typedef unsigned short Uint16;

typedef std::vector<Uint8> Uint8Sequence;
typedef std::vector<Uint16> Uint16Sequence;

enum PartID { HEAD, CHEST, WAIST, RFLEG, LFLEG, RHLEG, LHLEG };

struct Point
{
    double x;
    double y;
    Point(double X, double Y) : x(X), y(Y) {}
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
