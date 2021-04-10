#include <parts_lib/KondoServo.h>

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

KondoServo::KondoServo ( const KondoServo & original ) :
	id( original.id ),
	part_id( original.part_id ),
    degree( original.degree ),
    temp( original.temp ),
    speed( original.speed ),
    current( original.current ),
    stretch( original.stretch ),
    temp_limit( original.temp_limit ),
    current_limit( original.current_limit )
{}

PartID  KondoServo::get_part_id () const { return part_id; }
Uint8  KondoServo::get_id () const { return id; }
Uint16 KondoServo::get_degree () const { return degree; }
double KondoServo::get_degree_by_degree () const { return double(degree) * ( (135.0*2)/65535.0 - 135.0); }
Uint8 KondoServo::get_temp () const { return temp; }
Uint8 KondoServo::get_speed () const { return speed; }
Uint8 KondoServo::get_current() const { return current; }
Uint8 KondoServo::get_strech() const { return stretch;  }
Uint8 KondoServo::get_temp_limit() const { return temp_limit; }
Uint8 KondoServo::get_current_limit() const { return current_limit; }

void KondoServo::set_degree(Uint16 Degree) { degree = Degree; }
void KondoServo::set_temp(Uint8 Temp) { temp = Temp; }
void KondoServo::set_speed(Uint8 Speed) { speed = Speed;}
void KondoServo::set_current(Uint8 Current) { current = Current; }
void KondoServo::set_strech(Uint8 Stretch) { stretch = Stretch; }
void KondoServo::set_temp_limit(Uint8 Temp_limit) { temp_limit = Temp_limit; }
void KondoServo::set_current_limit(Uint8 Current_limit) { current_limit = Current_limit; }

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

ServoCommandMsg KondoServo::get_CommandMsg() const {
    ServoCommandMsg msg;
    set_CommandMsg( msg );
    return msg;
}

void KondoServo::set_CommandMsg( ServoCommandMsg & msg ) const {
    msg.id = id;
    msg.current_limit = current_limit;
    msg.degree = degree;
    msg.speed = speed;
    msg.stretch = stretch;
    msg.temp_limit = temp_limit;
}

void KondoServo::set( ServoCommandMsg & msg) {
    if( msg.id != id ) {
        ROS_INFO("Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, id);
        return;
    }
    set_current_limit( msg.current_limit );
    set_degree( msg.degree );
    set_speed( msg.speed );
    set_strech( msg.stretch );
    set_temp_limit( msg.temp_limit );
}

void KondoServo::set( typename ServoFeedbackMsg::ConstPtr & msg ) {
    if( msg->id != id ) {
        ROS_INFO("ERROR: Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg->id, id);
        return;
    }
    set_degree( msg->degree );
    set_current( msg->current );
    set_temp( msg->temp );
}

void KondoServo::set( ServoFeedbackMsg & msg ) {
    if( msg.id != id ) {
        ROS_INFO("ERROR: Could not set KondoServo from the msg since the ID does not match; object ID:[%d] and msg ID:[%d].", msg.id, id);
        return;
    }
    set_degree( msg.degree );
    set_current( msg.current );
    set_temp( msg.temp );
}
