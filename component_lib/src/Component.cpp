#include "component_lib.h"

Component::Component( PartId pId, ComnponentId cId, uint8_t num ){
    part_id = pId;
    component_id = cId;
    number = num;
    state = invalid;
}


Component::PartId get_part_id() const { return part_id; }
Component::uint8_t get_component_id() const { return component_id; }
Component::TransferableObjectState get_state() const { return state; }
Component::bool is_valid() const { return valid; }
