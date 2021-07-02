#ifndef COMPONENT_H
#define COMPONENT_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>


class Component: public TransferableObject {
	public:
		struct GlobalId{
			const PartId part_id;
			const ComponentId component_id;
			const IdNumber number;
		};
	protected:
		// id
		GlobalId( Component );

		// validness of the derived object. Will be set to false having had a serious error
		bool valid;

		// member functions
		Component( PartId, ComponentId, IdNumber, TransferableobjectState = NONE );
		PartId get_part_id() const ;
		ComponentId get_component_id() const ;
		IdNumber get_id_number() const ;
		bool is_valid() const ;

	public:
		virtual void print() const =0 ;
		Component( PartId, ComponentId, idNumber );
		CattyError check_id( Component );
		void print_id_error( CattyError, Component );
};

#include "component_lib_impl.h"


template <typename GeneralMsgType, typename CommandMsgType, typename FeedbackMsgType>
class Actuator: public Component
{
	protected:
		Actuator( PartId, ComponentId, IdNumber, TransferableObjectState = NONE );

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
	public:
		// struct definitions of the servo
		struct Command {
			uint16_t ideal_position;        // [0, 65535]<=>[-135, 135] at teensy translated into [3500, 11500]
			uint8_t temp_limit;       		// [1 ~ 127], 30 = 100[C], 75 = 70[C];
			uint8_t current_limit;    		// [1 ~ 63], I [A]
			uint8_t speed;            		// [1 ~ 127]
			uint8_t stretch;          		// [1 ~ 127]
			bool free;

			void operator=( const component_lib::KondoServoCommandMsg::ConstPtr& );
			void fill( component_lib::KondoServoCommandMsg& );
		};

		struct Feedback {
			uint16_t actual_position;
			uint8_t temp;             // [1 ~ 127], 30 = 100
			uint8_t current;          // [1 ~ 63], I [A]
			bool is_freed;

			void operator=( const component_lib::KondoServoFeedbackMsg::ConstPtr& );
			void fill( component_lib::KondoServoFeedbackMsg& );
		};

		struct Bounds {
			const uint16_t ideal_position_max = 0;
			const uint16_t ideal_position_min = 0;
			const uint8_t temp_limit_max = 0;
			const uint8_t temp_limit_min = 0;
			const uint8_t current_limit_max = 0;
			const uint8_t current_limit_min = 0;
			const uint8_t speed_max = 0;
			const uint8_t speed_min = 0;
			const uint8_t stretch_max = 0;
			const uint8_t stretch_min = 0;
		};

		struct ParametersForSpaceGeneration {
			const float max_speed=0;			// degree/sec
		};

    private:
		// command information
	    Command command;

		// feedback information
		Feedback feedback;

		// command field bounds
		Bounds bounds;

		// parameters for space generation
		ParametersForSpaceGeneration parameters_for_space_generation;


    public:
		// Initialization for controllers
        KondoServo( PartId, ComponentId, IdNumber, const Command& initial_position );		// For MotionController
		KondoServo( PartId, ComponentId, IdNumber );						// For FeedbackProcessor

		// Inicialization for learning node
		KondoServo(PartId, ComponentId, IdNumber, TransferableObjectState, Bounds, ParametersForSpaceGeneration);

		// Explicit copy constructor
        KondoServo ( const KondoServo & );

		// = oprator overloading
		void operator=( const KondoServo & );

		// access to the private members
        const Command& get_command_parameters();
		const Feedback& get_feedback_parameters();
		const Bounds& get_bounds();
		const ParametersForSpaceGeneration& get_parameters_for_space_generation();

        void set_command( const Command& command );
		void set_feedback( const Feedback& feedback );

		// printing information
        void print() const override ;

		// functions for message interaction
        void fill_command_msg( component_lib::KondoServoCommandMsg & ) const override ;
        void fill_feedback_msg( component_lib::KondoServoFeedbackMsg & ) const override ;
        void set( const component_lib::KondoServoCommandMsg::ConstPtr & ) override ;
        void set( const component_lib::KondoServoFeedbackMsg::ConstPtr & ) override ;
};


class Motor:
	public Actuator<
		component_lib::MotorGeneralMsg,
		component_lib::MotorCommand,
		component_lib::MotorFeedbackMsg
	>
{
	public:
		struct Command {
			uint16_t command_position;  // [0, 65535] where 0 = 65535
        	uint8_t current_limit;
			uint8_t responsiveness;           // [0, 255]

			void operator=( const component_lib::MotorCommandMsg::ConstPtr& );
			void fill( component_lib::MotorCommandMsg& );
		};

		struct Feedback {
			uint16_t actual_position;         // [0, 65535] where 0 = 65535
	        uint8_t temperature;              // centigrade
	        uint8_t current;

			void operator=( const component_lib::MotorFeedbackMsg::ConstPtr& );
			void fill( component_lib::MotorFeedbackMsg& );
		};

		struct Bounds {
			const uint16_t ideal_position_max = 0;
			const uint16_t ideal_position_min = 0;
			const uint8_t temp_limit_max = 0;
			const uint8_t temp_limit_min = 0;
			const uint8_t current_limit_max = 0;
			const uint8_t current_limit_min = 0;
			const uint8_t speed_max = 0;
			const uint8_t speed_min = 0;
			const uint8_t stretch_max = 0;
			const uint8_t stretch_min = 0;
		};

		struct ParametersForSpaceGeneration {
			const float max_speed = 0;			// degree/sec
		};

    private:
		// command information
        Command command;

		// feedback information
        Feedback feedback;



    public:
		// Initialization for controllers
        Motor( PartId, ComponentId, IdNumber, const Command& initial_position );		// For MotionController
		Motor( PartId, ComponentId, IdNumber );											// For FeedbackProcessor

		// Inicialization for learning node
		Motor( PartId, ComponentId, IdNumber, TransferableObjectState, Bounds, ParametersForSpaceGeneration);

		// Explicit copy constructor
        Motor( const Motor & );

        void operator=( const Motor & );

        const Command& get_command_parameters() const ;
		const Feedback& get_feedback_parameters() const ;
		const Bounds& get_bounds();
		const ParametersForSpaceGeneration& get_parameters_for_space_generation();

		void set_command( const Command& command );
		void set_feedback( const Feedback& feedback );

        void print() const override;

        void fill_command_msg( component_lib::MotorCommandMsg & ) const ;
        void fill_feedback_msg( component_lib::MotorFeedbackMsg & ) const ;

        void set( const component_lib::MotorCommandMsg::ConstPtr & );
        void set( const component_lib::MotorFeedbackMsg::ConstPtr & );
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
