#ifndef GENERIC_COMPONENT_H
#define GENERIC_COMPONENT_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>
#include <string>

class Component {
	protected:
		const PartID part_id;
		const Uint8 part_id;
		static
};


class GenericActuator
{
	protected:
		const PartID part_id;
		const Uint8 id;
		bool valid;                         // will be set to false having had a serious error

		virtual void print() const =0;

		GenericParts( PartID, Uint8 id );
		GenericParts( Msg & );

		check_msg_id();

	public:
		PartID get_part_id() const ;
		Uint8 get_id() const ;
		ObjectState get_state() const ;
		bool is_valid() const ;
};

#include<parts_lib/GenericPartsImpl.h>

#endif
