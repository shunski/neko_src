# KondoServoGeneralMsg.msg

component_lib/GenericComponentMsg component_property

# command information
uint8 current_limit
uint16 command_degree
uint8 speed
uint8 stretch
uint8 temp_limit
bool free

# feedback information
uint8 current
uint8 temp
bool feedback_degree
bool is_freed


# see the definitions in component.h
