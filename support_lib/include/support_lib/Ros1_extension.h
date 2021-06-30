#ifndef ROS1_EXTENSION_H
#define ROS1_EXTENSION_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>


// abstract class that holds the properties needed for the transfer.
// All transferable objects (by messages) must inherit this class.
class TransferableObject {
    private:
        ObjectState state = NONE;		     // will be set to either COMMAND, FEEDBACK or GENERAL as needed

    protected:
		TransferableObjectState get_state() const { return state; }
        template <typename MessageType>
        CattyError check_state( const typename MessageType::ConstPtr& message ){
            if constexpr( message_state<MessageType>::value == COMMAND ){
                if ( state != COMMAND ){
                    ROS_ERROR( "OBJECT_STATE_NOT_MATCH: State of Object is %s, but that of message is COMMAND", get_description( this->state ).c_str() );
                    return OBJECT_STATE_NOT_MATCH;
                }
            } else  if constexpr( message_state<MessageType>::value == FEEDBACK ){
                if ( state != FEEDBACK ){
                    ROS_ERROR( "OBJECT_STATE_NOT_MATCH: State of Object is %s, but that of message is FEEDBACK", get_description( this->state ).c_str() );
                    return OBJECT_STATE_NOT_MATCH;
                }
            }
            return SUCCESS;
        }
};

// trait that detects the message states (COMMAND, FEEDBACK or GENERAL)
template <typename T>
struct message_state {
    static const TransferableObjectState value = NONE;
};

// trait that detects transferable objects
// ALL of the following members must be specialized in each specialized class:
// - (public) CattyError fill_general_msg();
// - (public) CattyError fill_command_msg();
// - (public) CattyError fill_feedback_msg();
// and must inherit the specialized version of TransferableObject.
template< typename ObjectType >
struct is_transferable_object {
    static const bool value = false;
};

template<>
struct is_transferable_object<TransferableObject> {
    static const bool value = true;
}

// trait that detects component objects (motor, sensor, ...)
// this trait is implemented in component_lib
template< typename ObjectType >
struct get_component_kind {
    static const ComponentKind value = INVALID_COMPONENT_KIND;
};


// the neko_msg struct tells the information about the formula
template < typename ObjectType, typename MessageType >
struct neko_msg {
    static void set_msg( const ObjectType& object, MessageType& message ){
        if constexpr( is_transferable_object<ObjectType>::value && ros::message_traits::IsMessage::value ) {
            if constexpr ( message_state<MessageType>::value == COMMAND ){
                object.fill_command_msg( message );
            } else if constexpr( message_state<MessageType>::value == FEEDBACK ){
                object.fill_feedback_msg( message );
            } else if constexpr( message_state<MessageType>::value == GENERAL ){
                object.fill_feedback_msg( message );
            } else {
                static_assert("The message does not support the message_state<T> trait.");
            }
            object.update_state();
        } else if( is_transferable_object<ObjectType>::value ) {
            static_assert("The second argument is not a ros1 object message.");
        } else if( ros::message_traits::<MessageType>::value ) {
            static_assert("The first argument is not a transferable object.");
        } else {
            static_assert("Neither the first argument is not a transferable object nor the second argument is not a ros1 message.");
        }
    }

    static void set( ObjectType& object, const typename MessageType::ConstPtr& message_ptr ){
        if constexpr( is_transferable_object<ObjectType>::value && message_state<MessageType>::value!=NONE ) {
            object.set( message_ptr );
        } else if constexpr( is_transferable_object<ObjectType>::value ) {
            static_assert("The second argument is not a proper message.");
        } else if constexpr( message_state<MessageType>::value!=NONE ) {
            static_assert("The first argument is not a transferable object.");
        } else {
            static_assert("Neither the first argument is not a transferable object nor the second argument is not a ros1 message ConstPtr.");
        }
    }

    static CattyError check_msg_id( const typename MessageType::ConstPtr message_ptr ) {
        if constexpr( is_transferable_object<ObjectType>::value && message_state<MessageType>::value!=NONE ) {
            if ( object.get_part_id() != message_ptr->part_id ) return PART_ID_NOT_MATCH;
            if constexpr ( get_component_kind<ObjectType> ){  // the expression returns false if the type is INVALID_COMPONENT_KIND
                if ( object.get_component_id() != message_ptr->component_id ) return COMPONENT_ID_NOT_MATCH;
            }
            return SUCCESS;

        } else if constexpr( is_transferable_object<ObjectType>::value ) {
            static_assert("The second argument is not a proper message.");
        } else if constexpr( message_state<MessageType>::value　!=　NONE ) {
            static_assert("The first argument is not a transferable object.");
        } else {
            static_assert("Neither the first argument is not a transferable object nor the second argument is not a ros1 message ConstPtr.");
        }
    }
};
#endif
