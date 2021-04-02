// KondoServo.cpp implemented by KondoServo.cpp
#ifndef KONDOSERVO_H
#define KONDOSERVO_H

#include <ros/ros.h>
#include <Utilities.h>
#include <jcatty_parts_msgs/KondoServoCommandMsg.msg>
#include <jcatty_parts_msgs/KondoServoFeedbackMsg.msg>

typedef jcatty_parts_msgs::KondoServoCommandMsg ServoCommandMsg;
typedef jcatty_parts_msgs::KondoServoFeedbackMsg ServoFeedbackMsg;

class KondoServo
{
    private:
        PartID part_id;         // see definition in Utilities.h
        Uint8 id;               // [0, 31]
	    Uint16 degree;          // [0, 65535]<=>[-135, 135] at teensy tranlated into [3500, 11500]
	    Uint8 temp;             // [1 ~ 127], 30 = 100
	    Uint8 speed;            // [1 ~ 127]
	    Uint8 current;          // [1 ~ 63], I [A]
	    Uint8 stretch;          // [1 ~ 127]
        Uint8 temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
        Uint8 current_limit;    // [1 ~ 63], I [A]

    public:
        KondoServo ( Uint8 ID, PartID partID);
        KondoServo ( Uint8 ID, PartID partID, ServoCommandMsg & );
        KondoServo ( Uint8 ID, PartID partID, typename ServoFeedbackMsg::ConstPtr & );
        KondoServo ( Uint8 ID, PartID partID, ServoFeedbackMsg & );
        PartID  get_part_id () const 
        unsigned char get_id () const;
        unsigned short get_degree () const;
        double get_degree_by_degree () const;
        Uint8 get_temp () const;
        Uint8 get_speed () const;
        Uint8 get_current() const;
        Uint8 get_strech() const;
        Uint8 get_temp_limit() const;
        Uint8 get_current_limit() const;

        void set_degree( Uint16 Degree );
        void set_temp( Uint8 Temp );
        void set_speed( Uint8 Speed );
        void set_current( Uint8 Current );
        void set_strech( Uint8 Stretch );
        void set_temp_limit( Uint8 Temp_limit );
        void set_current_limit( Uint8 Current_limit );
        void print();

        ServoCommandMsg get_CommandMsg() const;
        void set_CommandMsg( ServoCommandMsg & msg ) const;
        void set( typename ServoFeedbackMsg::ConstPtr & msg );
        void set( ServoFeedbackMsg & msg );
};

#endif
