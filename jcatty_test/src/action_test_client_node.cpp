#include <motioncontroll_action/MotionControllAction.h>
#include <actionlib/client/simple_action_client.h>
#include <body_lib/Body.h>
#include <ros/ros.h>
#include <iostream>


// --------------------------- A class used for the test ---------------------------------------------------------

class ActionTestClientNode
{
    private:
        actionlib::SimpleActionClient<motioncontroll_action::MotionControllAction> test_client;
        int scene_number;

    public:
        ActionTestClientNode():
            test_client( "SomePartLocomotionAction", true )
        { test_client.waitForServer(); }

        void startTesting();
        void feedbackCallback( const motioncontroll_action::MotionControllFeedbackConstPtr& feedback );
        void resultCallback( const actionlib::SimpleClientGoalState& state,
                             const motioncontroll_action::MotionControllResultConstPtr& goal );

        motioncontroll_action::MotionControllGoal setUpTestAction();
};

// -------------------------------------------------------------------------------------------------------------------

// --------------------------- Class member function implementation -----------------------------------------------------

void ActionTestClientNode::startTesting(){
    motioncontroll_action::MotionControllGoal test_action_goal;
    test_action_goal = setUpTestAction();
    scene_number = 0;

    ROS_INFO( "Sending the action goal to the server..." );
    test_client.sendGoal( test_action_goal,
                          boost::bind( &ActionTestClientNode::resultCallback, this, _1, _2 ),
                          actionlib::SimpleActionClient<motioncontroll_action::MotionControllAction>::SimpleActiveCallback(),
                          boost::bind( &ActionTestClientNode::feedbackCallback, this, _1));
}


void ActionTestClientNode::feedbackCallback( const motioncontroll_action::MotionControllFeedbackConstPtr& feedback ) {
    while( scene_number < feedback->actualCurrentState.scene_id-1 ){
        ROS_INFO( "Missed Feedback scene of number \t:[%d]", scene_number );
        scene_number++;
    }
    ROS_INFO( "Received Feedback scene of number \t:[%d]", feedback->actualCurrentState.scene_id );
    scene_number++;
}


void ActionTestClientNode::resultCallback( const actionlib::SimpleClientGoalState& state,
                                           const motioncontroll_action::MotionControllResultConstPtr& goal ){
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Action Done. Shutting down...");
    ros::shutdown();
}


motioncontroll_action::MotionControllGoal ActionTestClientNode::setUpTestAction() {
    motioncontroll_action::MotionControllGoal goal;
    goal.part_id = RHLEG; // DO NOT CHANGE HERE

    int sequenceSize = rand() % 1000;
    double eachSceneDuration = ( rand() % 100 ) / 1000.0;

    goal.sequenceSize = sequenceSize;
    goal.expectedTotalDuration = ros::Duration( eachSceneDuration * sequenceSize );
    goal.partCommandSequence.reserve( sequenceSize );
    for (  int i=0; i < sequenceSize; ++i )
    {
        body_msgs::PartCommandMsg msg;
        body::Part(RHLEG).set_RandomPartCommandMsg( msg );
        goal.partCommandSequence.push_back( msg );
    }

    ROS_INFO("The action with the random sequence of size %d is set.", sequenceSize);

    return goal;
}

// ---------------------------------------------------------------------------------

// -------------------- main function -----------------------------------------------

int main( int argc, char** argv ){
    ros::init( argc, argv, "test_action");
    ActionTestClientNode node;

    // Testing
    node.startTesting();

    ros::spin();
    return 0;
}
