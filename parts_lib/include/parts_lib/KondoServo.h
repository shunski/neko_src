// KondoServo.h: implemented by KondoServo.cpp
#ifndef KONDOSERVO_H
#define KONDOSERVO_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_lib/GenericParts.h>
#include <parts_msgs/KondoServoCommandMsg.h>
#include <parts_msgs/KondoServoFeedbackMsg.h>
#include <parts_msgs/KondoServoMsg.h>

typedef parts_msgs::KondoServoCommandMsg ServoCommandMsg;
typedef parts_msgs::KondoServoFeedbackMsg ServoFeedbackMsg;
typedef parts_msgs::KondoServoMsg ServoMsg;

class KondoServo : GenericParts< ServoMsg, ServoCommandMsg, ServoFeedbackMsg >
{
    private:
	    Uint16 degree;          // [0, 65535]<=>[-135, 135] at teensy tranlated into [3500, 11500]
        Uint8 temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
        Uint8 current_limit;    // [1 ~ 63], I [A]
        Uint8 speed;            // [1 ~ 127]
        Uint8 stretch;          // [1 ~ 127]

	    Uint8 temp;             // [1 ~ 127], 30 = 100
	    Uint8 current;          // [1 ~ 63], I [A]

    public:
        KondoServo ( PartID pID, Uint8 ID );
        KondoServo ( const KondoServo & );

        KondoServo ( const ServoMsg & );
        KondoServo ( const typename ServoMsg::ConstPtr & );
        KondoServo ( const ServoCommandMsg & );
        KondoServo ( const typename ServoCommandMsg::ConstPtr & );
        KondoServo ( const ServoFeedbackMsg & );
        KondoServo ( const typename ServoFeedbackMsg::ConstPtr & );

        Uint16 get_degree () const;
        double get_degree_by_degree () const;
        Uint8 get_temp () const;
        Uint8 get_speed () const;
        Uint8 get_current() const;
        Uint8 get_strech() const;
        Uint8 get_temp_limit() const;
        Uint8 get_current_limit() const;

        void set_degree( const Uint16 Degree );
        void set_temp( const Uint8 Temp );
        void set_speed( const Uint8 Speed );
        void set_current( const Uint8 Current );
        void set_strech( const Uint8 Stretch );
        void set_temp_limit( const Uint8 Temp_limit );
        void set_current_limit( const Uint8 Current_limit );

        void print() const override ;

        CattyPartsError set_msg( ServoMsg & ) const override ;
        CattyPartsError set_CommandMsg( ServoCommandMsg & ) const override ;
        CattyPartsError set_FeedbackMsg( ServoFeedbackMsg & ) const override ;

        CattyPartsError set( const ServoMsg & ) override ;
        CattyPartsError set( const typename ServoMsg::ConstPtr & ) override  ;
        CattyPartsError set( const ServoCommandMsg & ) override ;
        CattyPartsError set( const typename ServoCommandMsg::ConstPtr & ) override ;
        CattyPartsError set( const ServoFeedbackMsg & ) override ;
        CattyPartsError set( const typename ServoFeedbackMsg::ConstPtr & ) override ;
};

#endif
