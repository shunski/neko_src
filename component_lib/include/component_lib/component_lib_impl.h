#ifndef COMPONENT_LIB_INPL_H
#define COMPONENT_LIB_IMPL_H

Component::Component( PartId part_id, ComponentId component_id, IdNumber id_number):
	part_id( part_id ),
	component_id( component_id ),
	number( id_number )
{}


Component::PartId get_part_id() const { return part_id; }
Component::ComponentId get_component_id() const { return component_id }
Component::IdNumber get_id_number() const { return id_number }
Component::bool is_valid() const { return valid; }

#endif
