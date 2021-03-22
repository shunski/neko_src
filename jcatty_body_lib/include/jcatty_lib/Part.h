// Part.h implemented by Part.cpp


#ifndef PART_H
#define PART_H

#include <iostream>
#include <string>
#include <vector>
#include "DCmotor.h"
#include "Servo.h"

using namespace std;

class Part
{
    protected:
        string name;
        vector<KondoServo> servo;
        vector<DCmotor> motor;

    public:
        Part ( string Name, unsigned char servo_num, unsigned char motor_num );
        vector<KondoServo> get_servo () const { return servo };
        vector<KondoServo> get_motor () const { return motor };
};


#endif
