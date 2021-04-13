#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_msgs/BrushlessMotorCommandMsg.h>
#include <parts_msgs/BrushlessMotorFeedbackMsg.h>
#include <parts_msgs/BrushlessMotorMsg.h>

typedef parts_msgs::BrushlessMotorCommandMsg BrushlessCommandMsg;
typedef parts_msgs::BrushlessMotorFeedbackMsg BrushlessFeedbackMsg;
typedef parts_msgs::BrushlessMotorMsg BrushlessMsg;

class BrushlessMotor : public Parts<BrushlessMsg, BrushlessCommandMsg, BrushlessFeedbackMsg>
{
	private:
		// Command
		Int16 voltage;         // -30,000~30,000

		// Feedback
		Uint16 position;       // 0~8191
		Int16 speed;           // [rpm]
		Uint16 current;

	public:
		BrushlessMotor( Uint8 ID, PartID, ObjectState );
		BrushlessMotor( BrushlessCommandMsg & );
		BrushlessMotor( BrushlessFeedbackMsg & );
		BrushlessMotor( typename BrushlessFeedbackMsg & );
		Uint16 get_id() const ;
		Uint16 get_part_id() const ;
		Int16 get_voltage() const ;
		Uint16 get_position() const ;
		Int16 get_speed() const ;
		Uint16 get_current() const ;

		CattyPartsError set_voltage( Uint16 );
		CattyPartsError set_position( Uint16 );
		CattyPartsError set_speed( Uint16 );
		CattyPartsError set_current( Uint16 );
		
		void print() const override;

		CattyPartsError set( BrushlessMsg & ) override ;
		CattyPartsError set( typename BrushlessMsg::ConstPtr & ) override ;
		CattyPartsError set( BrushlessCommandMsg & ) override ;
		CattyPartsError set( typename BrushlessCommandMsg::ConstPtr & ) override ;
		CattyPartsError set( BrushlessFeedbackMsg & ) override ;
		CattyPartsError set( typename ServoFeedbackMsg::ConstPtr & ) override ;

		CattyPartsError set_msg( BrushlessMsg & ) const override ;
		CattyPartsError set_CommandMsg( BrushlessCommandMsg & ) const override ;
		CattyPartsError set_FeedbackMsg( BrushlessFeedbackMsg & ) const override ;
};

#endif
