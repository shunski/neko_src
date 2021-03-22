// Plot.h implemented by  Plot.cpp

#ifndef PLOT_H
#define PLOT_H

struct Point
{
    float x;
    float y;
    Point(float X, float Y) : x(X), y(Y) {}
};

class Plot
{
    private:
        Vector<Point> points;      // stored in such a way that the Point.x is an increasing order
        float x_max;
        float y_max;
        float y_min;
    public:
        Plot() : x_max(0), y_max(0), y_min(0) {}
        vector<Point> get_points(){ return points }
        float get_xMax() const { return x_max }
        float get_yMax() const { return y_max }
        float get_yMin() const { return y_min }
        bool add_point( Point p );

};
