#ifndef ACTUATOR_MSGS_EXTENSION_H
#define ACTUATOR_MSGS_EXTENSION_H

#include <support_lib/Utilities.h>
#include <actuator_msgs.h>

// KondoServo messages definitions
template message_type< KondoServoCommandMsg > {
    static const TransterableObjectMessage value = COMMAND;
};

template message_type< KondoServoFeedbackMsg > {
    static const TransterableObjectMessage value = FEEDBACK;
};

template message_type< KondoServoGeneralMsg > {
    static const TransterableObjectMessage value = GENERAL;
};

// Motor messages definitions
template message_type< MotorServoCommandMsg > {
    static const TransterableObjectMessage value = COMMAND;
};

template message_type< MotorServoFeedbackMsg > {
    static const TransterableObjectMessage value = FEEDBACK;
};

template message_type< MotorServoGeneralMsg > {
    static const TransterableObjectMessage value = GENERAL;
};

#endif
