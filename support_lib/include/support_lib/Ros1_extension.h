#ifndef ROS1_EXTENSION_H
#define ROS1_EXTENSION_H

#include <ros/ros.h>
#include <support_lib/Utilities.h>


// abstract class that holds the properties needed for transfer
// All transferable (as messages) objeects must inherit this class
template <Typename MessageType>
class TransferableObject {
    protected:
        ObjectState state = INVALID;		// will be set to either COMMAND, FEEDBACK or GENERAL as needed
        bool well_defined;                  // will be set to true after enough information (as a particular 'state') filled out

        void update_state( const typename MessageType::ConstPtr& message ){
            if()
        }
};

// trait that detects the message type (COMMAND, FEEDBACK or GENERAL)
template <typename T>
struct message_state {
    static const TransferableObjectState value = INVALID;
};

// trait that detects transferble objects
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
struct is_transferable_objeect<TransferableObject>{
    static const bool value = false;
}


template < typename ObjectType, typename MessageType >
struct Ros1MessageInteraction {
    static void set_msg( const ObjectType& object, MessageType& message ){
        if constexpr( is_transferable_object<ObjectType>::value && ros::message_traits::IsMessage::value ) {
            if constexpr ( message_state<MessageType>::value == COMMAND ){
                object.set_command_msg( message );
            } else if constexpr( message_state<MessageType>::value == FEEDBACK ){
                object.set_feedback_msg( message );
            } else if constexpr( message_state<MessageType>::value == GENERAL ){
                object.set_feedback_msg( message );
            } else {
                static_assert("The message does not support the message_state<T> trait.");
            }
        } else if( is_transferable_object<ObjectType>::value ) {
            assert("The second argument is not a ros1 message.");
        } else if( ros::message_traits::<MessageType>::value ) {
            assert("The first argument is not a transferable object.");
        } else {
            assert("Neither the first argument is not a transferable object nor the second argument is not a ros1 message.");
        }
    }

    static void set( ObjectType& object, const typename MessageType::ConstPtr& message ){
        if constexpr( is_transferable_object<ObjectType>::value && is_ros1_message_ptr<MessageType>::value ) {
            object.set( message );
        } else if( is_transferable_object<ObjectType>::value ) {
            assert("The second argument is not a ros1 message ConstPtr.");
        } else if( ros::message_traits::IsMesssage<MessageType>::value ) {
            assert("The first argument is not a transferable object.");
        } else {
            assert("Neither the first argument is not a transferable object nor the second argument is not a ros1 message ConstPtr.");
        }
    }

    CattyError check_msg_id( const ObjectType object, const MessageType message ) const {
        if constexpr( is_transferable_object<ObjectType>::value && is_ros1_message_ptr::value ) {
            if ( object.get_part_id() != message->part_id) return PART_ID_NOT_MATCH;
            if ( object.get_id() != message->id) return ID_NOT_MATCH;
        } else {
            assert("Either the first object is not a transferable object as ros1 or the second argument is not a ros1 message.");
        }
    }
};

#endif
