// Body.h

#ifndef BODY_H
#define BODY_H

#include <ros/ros.h>
#include <CattySpecificPart.h>

#define head_servo_num 1;
#define head_motor_num 0;

#define chest_servo_num 1;
#define cheat_motor_num 0;

#define waist_servo_num 1;
#define waist_motor_num 1;

#define rf_servo_num 2;
#define rf_motor_num 2;

#define lf_servo_num 2;
#define lf_motor_num 2;

#define rh_servo_num 3;
#define rh_motor_num 1;

#define lh_servo_num 3;
#define lh_motor_num 1;


class Body
{
    private:
        Part right_fore;
        Part left_fore;
        Part right_hind;
        Part left_hind;
        Part waist;
        Part chest;
        Part head;

    public:
        Body() :
            head("Head", head_servo_num, head_motor_num),
            head("Chest", chest_servo_num, chest_motor_num),
            head("Waist", waist_servo_num, waist_motor_num),
            head("RightFore", rf_servo_num, rf_motor_num),
            head("LeftFore", lf_servo_num, lf_motor_num),
            head("RightHind", rh_servo_num, rh_motor_num),
            head("LeftHind", lh_servo_num, lh_motor_num)
        {}


};


#endif
