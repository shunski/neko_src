#ifndef COMPONENT_H
#define COMPONENT_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>

class Component {
	protected:
		// identities
		const PartId part_id;
		const ComponentId component_id;
		const uint8_t number;				// identifies compoents that has the same PartId and ComponentId

		// validness of object. Will be set to false having had a serious error
		bool valid;

		// member functions
		Component( PartId, ComnponentId, uint8_t );
		PartId get_part_id() const ;
		uint8_t get_component_id() const ;
		TransferableObjectState get_state() const ;
		bool is_valid() const ;
};


template<>
static is_component<Component>{
	static const bool value = true;
};


class GenericActuator: public Component
{
	protected:

		virtual void print() const =0;
		GenericActuator( PartID, Uint8 id );
};


class KondoServo: public GenericActuator
{
    private:
		// command information
	    Uint16 command_degree;          // [0, 65535]<=>[-135, 135] at teensy translated into [3500, 11500]
        Uint8 temp_limit;       // [1 ~ 127], 30 = 100[C], 75 = 70[C];
        Uint8 current_limit;    // [1 ~ 63], I [A]
        Uint8 speed;            // [1 ~ 127]
        Uint8 stretch;          // [1 ~ 127]
        bool free;

		// feedback information
        Uint16 feedback_degree;
	    Uint8 temp;             // [1 ~ 127], 30 = 100
	    Uint8 current;          // [1 ~ 63], I [A]
        bool is_freed;

		// field bounds
		const static uint16_t degree_max = 65535;
		const static uint16_t degree_min = 0;
		const static uint8_t temp_limit_max = 127;
		const static uint8_t temp_limit_min = 1;
		const static uint8_t current_limit_max = 63;
		const static uint8_t current_limit_min = 1;
		const static uint8_t speed_max = 127;
		const static uint8_t speed_min = 1;
		const static uint8_t stretch_max = 127;
		const static uint8_t stretch_min = 1;


    public:
        KondoServo ( PartId pID, Uint8 ID );
        KondoServo ( const KondoServo & );

        KondoServo ( const typename ServoMsg::ConstPtr & );
        KondoServo ( const typename ServoCommandMsg::ConstPtr & );
        KondoServo ( const typename ServoFeedbackMsg::ConstPtr & );

		void operator=( const KondoServo & );

        uint16_t get_command_degree () const;
        uint16_t get_feedback_degree () const;
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

        CattyError fill_general_msg( ServoMsg & ) const override ;
        CattyError fill_command_msg( ServoCommandMsg & ) const override ;
        CattyError fill_feedback_msg( ServoFeedbackMsg & ) const override ;

        CattyError set( const typename ServoMsg::ConstPtr & ) override  ;
        CattyError set( const typename ServoCommandMsg::ConstPtr & ) override ;
        CattyError set( const typename ServoFeedbackMsg::ConstPtr & ) override ;
};


class Motor : public GenericActuator
{

    private:
		// command information
        vector<uint16_t> ideal_position;  // [0, 65535] where 0 = 65535
        uint8_t current_limit;
		uint8_t responsiveness;           // [0, 255]

		// feedback information
        uint16_t actual_position;         // [0, 65535] where 0 = 65535
        uint8_t temperature;              // centigrade
        uint8_t current;

        static const uint8_t temperature_max;
		static const uint8_t current_max;
		static const uint16_t

    public:
        Motor( PartId, Uint8 component_id );
        Motor( const BrushedMotor & );

        Motor( const typename MotorMsg::ConstPtr & );
        Motor( const typename MotorCommandMsg::ConstPtr & );
        Motor( const typename MotorFeedbackMsg::ConstPtr & );

        void operator=( const BrushedMotor & );

        uint16_t get_rpm() const ;
        int16_t get_current_limit() const ;

        int16_t get_actual_position() const ;
        uint8_t get_temperature() const ;
        uint16_t get_current() const ;

        const map<ros::Time, int16_t>& get_actual_positions() const ;
        const map<ros::Time, int16_t>& get_current_drawed() const ;

        void print() const override;

        CattyError fill_general_msg( MotorGeneralMsg & ) const ;
        CattyError fill_command_msg( MotorCommandMsg & ) const ;
        CattyError fill_feedback_msg( MotorFeedbackMsg & ) const ;

        CattyError set( const typename MotorGeneralMsg::ConstPtr & );
        CattyError set( const typename MotorCommandMsg::ConstPtr & );
        CattyError set( const typename MotorFeedbackMsg::ConstPtr & );
};


#include<parts_lib/ComponentImpl.h>

#endif
