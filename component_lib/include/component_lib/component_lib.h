#ifndef COMPONENT_H
#define COMPONENT_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>


class Component: public TransferableObject {
	protected:
		// identities
		const PartId part_id;
		const ComponentId component_id;
		const IdNumber number;

		// validness of the derived object. Will be set to false having had a serious error
		bool valid;

		// member functions
		Component( PartId, ComponentId, IdNumber );
		PartId get_part_id() const ;
		ComponentId get_component_id() const ;
		IdNumber get_id_number() const ;
		bool is_valid() const ;

	public:
		virtual void print() const =0 ;

};

#include "component_lib_impl.h"


template <typename GeneralMsgType, typename CommandMsgType, typename FeedbackMsgType>
class Actuator: public Component
{
	protected:
		Actuator( PartId, IdNumber, uint16_t home_ideal_position );

	public:

		// functions that the derived classes need to implement
		virtual CattyError fill_general_msg( GeneralMsgType & ) const =0 ;
        virtual CattyError fill_command_msg( CommandMsgType & ) const =0 ;
        virtual CattyError fill_feedback_msg( FeedbackMsgType & ) const =0 ;

        virtual CattyError set( const typename GeneralMsgType::ConstPtr & ) =0 ;
        virtual CattyError set( const typename CommandMsgType::ConstPtr & ) =0 ;
        virtual CattyError set( const typename FeedbackMsgType::ConstPtr & ) =0 ;
};

template<>
struct get_component_kind<Actuator> {
	static const ComponentKind value = ACTUATOR;
};


class KondoServo
	: public Actuator<
		component_lib::KondoServoGeneralMsg,
		component_lib::KondoServoCommandMsg,
		component_lib::KondoServoFeedbackMsg
	>
{
    private:
		// command information
	    Uint16 ideal_position;          // [0, 65535]<=>[-135, 135] at teensy translated into [3500, 11500]
        Uint8 temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
        Uint8 current_limit;    // [1 ~ 63], I [A]
        Uint8 speed;            // [1 ~ 127]
        Uint8 stretch;          // [1 ~ 127]
        bool free;

		// feedback information
        Uint16 actual_position;
	    Uint8 temp;             // [1 ~ 127], 30 = 100
	    Uint8 current;          // [1 ~ 63], I [A]
        bool is_freed;

		// command field bounds
		const static uint16_t ideal_position_max = 65535;
		const static uint16_t ideal_position_min = 0;
		const static uint8_t temp_limit_max = 127;
		const static uint8_t temp_limit_min = 1;
		const static uint8_t current_limit_max = 63;
		const static uint8_t current_limit_min = 1;
		const static uint8_t speed_max = 127;
		const static uint8_t speed_min = 1;
		const static uint8_t stretch_max = 127;
		const static uint8_t stretch_min = 1;


    public:
        KondoServo ( PartId partId, Uint8 number );
        KondoServo ( const KondoServo & );

        KondoServo ( const component_lib::KondoServoGeneralMsg::ConstPtr & );
        KondoServo ( const component_lib::KondoServoCommandMsg::ConstPtr & );
        KondoServo ( const component_lib::KondoServoFeedbackMsg::ConstPtr & );

		void operator=( const KondoServo & );

        uint16_t get_ideal_position () const;
        uint16_t get_actual_position () const;
        uint8_t get_temp () const;
        uint8_t get_speed () const;
        uint8_t get_current() const;
        uint8_t get_stretch() const;
        uint8_t get_temp_limit() const;
        uint8_t get_current_limit() const;

        void set_command_degree( const uint16_t Degree );
        void set_temp( const uint8_t Temp );
        void set_speed( const uint8_t Speed );
        void set_current( const uint8_t Current );
        void set_stretch( const uint8_t Stretch );
        void set_temp_limit( const uint8_t Temp_limit );
        void set_current_limit( const uint8_t Current_limit );

        void print() const override ;

        CattyError fill_command_msg( component_lib::KondoServoCommandMsg & ) const override ;
        CattyError fill_feedback_msg( component_lib::KondoServoFeedbackMsg & ) const override ;

        CattyError set( const component_lib::KondoServoCommandMsg::ConstPtr & ) override ;
        CattyError set( const component_lib::KondoServoFeedbackMsg::ConstPtr & ) override ;
};


class Motor:
	public Actuator<
		component_lib::MotorGeneralMsg,
		component_lib::MotorCommand,
		component_lib::MotorFeedbackMsg
	>
{

    private:
		// command information
        vector<uint16_t> command_position;  // [0, 65535] where 0 = 65535
        uint8_t current_limit;
		uint8_t responsiveness;           // [0, 255]

		// feedback information
        uint16_t actual_position;         // [0, 65535] where 0 = 65535
        uint8_t temperature;              // centigrade
        uint8_t current;

		// command field bounds
        static const uint8_t temperature_max;
		static const uint8_t current_max;

    public:
        Motor( PartId, Uint8 component_id );
        Motor( const Motor & );

        Motor( const component_lib::MotorMsg::ConstPtr & );
        Motor( const component_lib::MotorCommandMsg::ConstPtr & );
        Motor( const component_lib::MotorFeedbackMsg::ConstPtr & );

        void operator=( const Motor & );

        uint16_t get_rpm() const ;
        int16_t get_current_limit() const ;

        int16_t get_actual_position() const ;
        uint8_t get_temperature() const ;
        uint16_t get_current() const ;

        const map<ros::Time, int16_t>& get_actual_positions() const ;
        const map<ros::Time, int16_t>& get_current_drawed() const ;

        void print() const override;

        CattyError fill_general_msg( component_lib::MotorGeneralMsg & ) const ;
        CattyError fill_command_msg( component_lib::MotorCommandMsg & ) const ;
        CattyError fill_feedback_msg( component_lib::MotorFeedbackMsg & ) const ;

        CattyError set( const component_lib::MotorGeneralMsg::ConstPtr & );
        CattyError set( const component_lib::MotorCommandMsg::ConstPtr & );
        CattyError set( const component_lib::MotorFeedbackMsg::ConstPtr & );
};


class Sensor: public Component
{
	protected:
		Sensor( PartId, Uint8 Number );

	public:
        CattyError fill_feedback_msg( FeedbackMsgType & ) const ;

        CattyError set( const typename GeneralMsgType::ConstPtr & );
        CattyError set( const typename FeedbackMsgType::ConstPtr & );
};


template<>
struct get_component_kind<Sensor> {
	static const ComponentKind value = SENSOR;
};


class MotionSensor: public Sensor
{
	private:
		CattyError
};


#include<component_lib/ComponentImpl.h>

#endif
