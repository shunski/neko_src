/*
 * Part.cpp implementes Part.h
 * protected member: string name; vector<KnodoServo> servo; vector<DCmotor> motor;
*/


#include "Part.h"

using namespace std;

Part::Part ( string Name, unsigned char servo_num, unsigned char motor_num ) : name(Name){

    // initializing vector<KondoServo> servo
    for (unsigned char i=0; i<servo_num; i++) {
        KondoServo servo_motor(i);
        vector.push_back(servo_motor);
    }

    // initializing vector<DCmotor> motor
    for (unsigned char i=0; i<servo_num; i++) {
        DCmotor dcMotor(i);
        vector.push_back(dcMotor);
    }
}
