#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <parts_lib/GenericParts.h>
#include <parts_msgs/BrushlessMotorCommandMsg.h>
#include <parts_msgs/BrushlessMotorFeedbackMsg.h>
#include <parts_msgs/BrushlessMotorMsg.h>

typedef parts_msgs::BrushlessMotorCommandMsg BrushlessCommandMsg;
typedef parts_msgs::BrushlessMotorFeedbackMsg BrushlessFeedbackMsg;
typedef parts_msgs::BrushlessMotorMsg BrushlessMsg;

class BrushlessMotor : public GnericParts<BrushlessMsg, BrushlessCommandMsg, BrushlessFeedbackMsg>
{
	private:
		// Command
		Int16 voltage;         // -30,000~30,000

		// Feedback
		Uint16 position;       // 0~8191
		Int16 speed;           // [rpm]
		Uint16 current;

	public:
		BrushlessMotor( PartID, Uint8 ID );
		BrushlessMotor( const BrushlessMotor & );

		BrushlessMotor( const BrushlessMsg & );
		BrushlessMotor( const typename BrushlessMsg::ConstPtr & );
		BrushlessMotor( const BrushlessCommandMsg & );
		BrushlessMotor( const typename BrushlessCommandMsg::ConstPtr & );
		BrushlessMotor( const BrushlessFeedbackMsg & );
		BrushlessMotor( const typename BrushlessFeedbackMsg & );

		Int16 get_voltage() const ;
		Uint16 get_position() const ;
		Int16 get_speed() const ;
		Uint16 get_current() const ;

		CattyPartsError set_voltage( const Uint16 );
		CattyPartsError set_position( const Uint16 );
		CattyPartsError set_speed( const Uint16 );
		CattyPartsError set_current( const Uint16 );

		void print() const override;

		CattyPartsError set( const BrushlessMsg & ) override ;
		CattyPartsError set( const typename BrushlessMsg::ConstPtr & ) override ;
		CattyPartsError set( const BrushlessCommandMsg & ) override ;
		CattyPartsError set( const typename BrushlessCommandMsg::ConstPtr & ) override ;
		CattyPartsError set( const BrushlessFeedbackMsg & ) override ;
		CattyPartsError set( const typename ServoFeedbackMsg::ConstPtr & ) override ;

		CattyPartsError set_msg( BrushlessMsg & ) const override ;
		CattyPartsError set_CommandMsg( BrushlessCommandMsg & ) const override ;
		CattyPartsError set_FeedbackMsg( BrushlessFeedbackMsg & ) const override ;
};

#endif
