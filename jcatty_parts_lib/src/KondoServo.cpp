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

KondoServo::KondoServo ( Uint8 ID, PartID partID) :
    part_id(partID),
    id (ID),
    degree (128/2),                // Neutral
    temp (1),
    speed (90),
    current (1),
    stretch (60),
    temp_limit (75),
    current_limit (40)
{}

KondoServo::KondoServo ( Uint8 ID, PartID partID, ServoCommandMsg &  msg ):
    id(ID),
    part_id(partID)
{
    this->set(msg);
}

KondoServo::KondoServo ( Uint8 ID, PartID partID, typename ServoFeedbackMsg::ConstPtr & msg ) :
    id(ID),
    part_id(partID)
{
    this->set(msg);
}

KondoServo::KondoServo ( Uint8 ID, PartID partID, ServoFeedbackMsg & msg ):
    id(ID),
    part_id(partID)
{
    this->set(msg);
}

PartID  KondoServo::get_part_id () const { return part_id; }
Uint8  KondoServo::get_id () const { return id; }
Uin16 KondoServo::get_degree () const { return degree; }
double KondoServo::get_degree_by_degree () const { return double(degree) * ( (135.0*2)/65535.0 - 135.0); }
Uint8 KondoServo::get_temp () const { return temp; }
Uint8 KondoServo::get_speed () const { return speed; }
Uint8 KondoServo::get_current() const { return current; }
Uint8 KondoServo::get_strech() const { return stretch;  }
Uint8 KondoServo::get_temp_limit() const { return temp_limit; }
Uint8 KondoServo::get_current_limit() const { return current_limit; }

void KondoServo::set_degree(unsigned char Degree) { degree = Degree; }
void KondoServo::set_temp(unsigned char Temp) { temp = Temp; }
void KondoServo::set_speed(unsigned char Speed) { speed = Speed;}
void KondoServo::set_current(unsigned char Current) { current = Current; }
void KondoServo::set_strech(unsigned char Stretch) { stretch = Stretch; }
void KondoServo::set_temp_limit(unsigned char Temp_limit) { temp_limit = Temp_limit; }
void KondoServo::set_current_limit(unsigned char Current_limit) { current_limit = Current_limit; }

void KondoServo::print() {
    ROS_INFO("Printing Information of Servo: %d", id);
    ROS_INFO("\t degree = %d", degree);
    ROS_INFO("\t temp = %d", temp);
    ROS_INFO("\t speed = %d", speed);
    ROS_INFO("\t current = %d", current);
    ROS_INFO("\t stretch = %d", stretch);
    ROS_INFO("\t temp_limit = %d", temp_limit);
    ROS_INFO("\t current_limit = %d", current_limit);
}

CommandMsg KondoServo::get_CommandMsg() const {
    CommandMsg msg;
    set_CommandMsg();
    return msg;
}

void KondoServo::set_CommandMsg( CommandMsg & msg ) const {
    msg.id = this.get_id();
    msg.current_limit = this->get_current_limit();
    msg.degree = this->get_degree();
    msg.speed = this->get_speed();
    msg.stretch = this->get_strech();
    msg.temp_limit = this->get_temp_limit();
}

void set( CommandMsg & msg) {
    if( msg.id != this.get_id() ) {
        ROS_INFO("Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, this.get_Id());
        return;
    }
    set_current_limit( msg.current_limit );
    set_degree( msg.degree );
    set_speed( msg.speed );
    set_strech( msg.stretch );
    set_temp_limit( msg.temp_limit );
}

void KondoServo::set( typename FeedbackMsg::ConstPtr & msg ) {
    if( msg->id != this.get_id() ) {
        ROS_INFO("Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg->id, this.get_Id());
        return;
    }
    set_degree( msg->degree );
    set_current( msg->current );
    set_temp( msg->temp );
}

void KondoServo::set( FeedbackMsg & msg ) {
    if( msg.id != this.get_id() ) {
        ROS_INFO("Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, this.get_Id());
        return;
    }
    set_degree( msg.degree );
    set_current( msg.current );
    set_temp( msg.temp );
}
