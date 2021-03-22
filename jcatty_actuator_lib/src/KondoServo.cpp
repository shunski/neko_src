/* 
 * KondoServo.cpp implements KondoServo.h
 * unsigned char number;           // [0, 31]
 * unsigned char degree;           // [0, 127], 0 = -135, 127 = 135 tranlated into [3500, 11500]
 * unsigned char temp;             // [1 ~ 127], 30 = 100
 * unsigned char speed;            // [1 ~ 127]
 * unsigned char current;          // [1 ~ 63], I = I[A]
 * unsigned char strech;           // [1 ~ 127]
 * unsigned char temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
 * unsigned char current_limit;    // [1 ~ 63], I = I[A]
*/

#include "KondoServo.h"

KondoServo::KondoServo (unsigned char Number) {

    number = Number;
    degree = 128/2;                // Neutral
    temp = 1;
    speed = 90;
    current = 1;
    stretch = 60;
    temp_limit = 75;
    current_limit = 40;

}


void KondoServo::update (unsigned char Degree, unsigned char Temp, unsigned char Current){
    set_degree(Degree);
    set_temp(Temp);
    set_current(Current);
}


void Kondo_Servo::set (unsigned char Speed, unsigned char Stretch, unsigned char Temp_limit, unsigned char Current_limit) {
    set_speed(Speed);
    set_stretch(Stretch);
    set_temp_limit(Temp_limit);
    set_current_limit(Current_limit);
}


unsigned char  KondoServo::get_number () const { return number; }
unsigned char KondoServo::get_degree () const { return degree; }
unsigned char KondoServo::get_temp () const { return temp; }
unsigned char KondoServo::get_speed () const { return speed; }
unsigned char KondoServo::get_current() const { return current; }
unsigned char KondoServo::get_strech() const { return stretch;  }
unsigned char KondoServo::get_temp_limit() const { return temp_limit; }
unsigned char KondoServo::get_current_limit() const { return current_limit; }


void KondoServo::set_degree(unsigned char Degree) { degree = Degree; }
void KondoServo::set_temp(unsigned char Temp) { temp = Temp; }
void KondoServo::set_speed(unsigned char Speed) { speed = Speed;}
void KondoServo::set_current(unsigned char Current) { current = Current; }
void KondoServo::set_strech(unsigned char Stretch) { stretch = Stretch; }
void KondoServo::set_temp_limit(unsigned char Temp_limit) { temp_limit = Temp_limit; }
void KondoServo::set_current_limit(unsigned char Current_limit) { current_limit = Current_limit; }

void print() {
    ROS_INFO("Printing Information of Servo: %d", number);
    ROS_INFO("\t degree = %d", degree);
    ROS_INFO("\t temp = %d", temp);
    ROS_INFO("\t speed = %d", speed);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t stretch = %d", stretch);
    ROS_INFO("\t temp_limit = %d", temp_limit);
    ROS_INFO("\t current_limit = %d", current_limit);
}
