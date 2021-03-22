/* Plot.cpp implements Plot.h
 * private member: vector<Point> points
 *                 float x_max;
 *                 float y_max;
 *                 float y_min;
 */

bool Plot::add_point( Point p ){
    if( p.x < x_max ) return false;
    x_max = p.x;

    if( p.y > y_max ) y_max = p.y;
    if( p.y < y_min ) y_min = p.y;

    points.push_back(p);
    return true;
}
