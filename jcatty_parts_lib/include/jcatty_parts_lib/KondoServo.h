// KondoServo.cpp implemented by KondoServo.cpp


#ifndef KONDOSERVO_H
#define KONDOSERVO_H


#include <ros/ros.h>


class KondoServo
{

    private:
        unsigned char number;           // [0, 31]
	      unsigned char degree;           // [0, 127], 0 = -135, 127 = 135 tranlated into [3500, 11500]
	      unsigned char temp;             // [1 ~ 127], 30 = 100
	      unsigned char speed;            // [1 ~ 127]
	      unsigned char current;          // [1 ~ 63], I = I[A]
	      unsigned char stretch;          // [1 ~ 127]
        unsigned char temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
        unsigned char current_limit;    // [1 ~ 63], I = I[A]

    public:
        KondoServo (unsigned char Number);
        void update (unsigned char Degree, unsigned char Temp, unsigned char Current);
        void set (unsigned char Speed, unsigned char Stretch, unsigned char Temp_limit, unsigned char Current_limit);
        unsigned char get_number () const;
        unsigned char get_degree () const;
        unsigned char get_temp () const;
        unsigned char get_speed () const;
        unsigned char get_current() const;
        unsigned char get_strech() const;
        unsigned char get_temp_limit() const;
        unsigned char get_current_limit() const;

        void set_degree(unsigned char Degree);
        void set_temp(unsigned char Temp);
        void set_speed(unsigned char Speed);
        void set_current(unsigned char Current);
        void set_strech(unsigned char Stretch);
        void set_temp_limit(unsigned char Temp_limit);
        void set_current_limit(unsigned char Current_limit);
        void print();
};


#endif
