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

        void startTesting( int test_number );
        void feedbackCallback( const motioncontroll_action::MotionControllFeedbackConstPtr& feedback );
        void resultCallback( const actionlib::SimpleClientGoalState& state,
                             const motioncontroll_action::MotionControllResultConstPtr& goal );

        motioncontroll_action::MotionControllGoal setUpTestAction( int test_number );
};

// -------------------------------------------------------------------------------------------------------------------

// --------------------------- Class member function implementation -----------------------------------------------------

void ActionTestClientNode::startTesting( int test_number ){
    ROS_INFO( "Setting up the action of #%d...", test_number );
    motioncontroll_action::MotionControllGoal test_action_goal;
    test_action_goal = setUpTestAction( test_number );
    scene_number = 0;

    ROS_INFO( "Sending the action goal to the server..." );
    test_client.sendGoal( test_action_goal,
                          boost::bind( &ActionTestClientNode::resultCallback, this, _1, _2 ),
                          actionlib::SimpleActionClient<motioncontroll_action::MotionControllAction>::SimpleActiveCallback(),
                          boost::bind( &ActionTestClientNode::feedbackCallback, this, _1));
}


void ActionTestClientNode::feedbackCallback( const motioncontroll_action::MotionControllFeedbackConstPtr& feedback ) {
    while( scene_number < feedback->actualCurrentState.scene_id-1 ){
        ROS_INFO( "Missed Feedback scene of number \t:[%d]", feedback->actualCurrentState.scene_id );
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


motioncontroll_action::MotionControllGoal ActionTestClientNode::setUpTestAction( int test_number ){
    motioncontroll_action::MotionControllGoal goal;
    goal.part_id = RHLEG; // DO NOT CHANGE HERE

    int sequenceSize;
    double eachSceneDuration;

    switch( test_number )
    {
        case 0: // custom test
        {
            sequenceSize = 100;
            eachSceneDuration = 0.05; // can be changed. but not too small
            break;
        }

        case 1: // random test
        {
            sequenceSize = rand() % 1000;
            eachSceneDuration = ( rand() % 100 ) / 1000.0;
            break;
        }

        default:
        {
            ROS_INFO("Something went wrong. Could not set up the action");
            break;
        }
    }
    goal.sequenceSize = sequenceSize;
    goal.expectedTotalDuration = ros::Duration( eachSceneDuration * sequenceSize );
    goal.partCommandSequence.reserve( sequenceSize );
    for (  int i=0; i < sequenceSize; ++i )
    {
        body_msgs::PartCommandMsg msg;
        body::Part(RHLEG).set_RandomPartCommandMsg( msg );
        goal.partCommandSequence.push_back( msg );
    }

    ROS_INFO("The action with the sequence of size %d is set.", sequenceSize);

    return goal;
}

// ---------------------------------------------------------------------------------

// -------------------- main function -----------------------------------------------

int main( int argc, char** argv ){
    ros::init( argc, argv, "test_action");
    ActionTestClientNode node;


    // Getting user input for testing number
    std::string test_number_str;
    std::cout << "Inuput test number:" << std::endl << "\t0: custom action" << std::endl << "\t1: random action" << std::endl;
    getline( std::cin, test_number_str );
    while( test_number_str!="0" && test_number_str!="1" ){
        std::cout << "Oops! Your input does not seem to be correct... Try Again." << std::endl;
        getline( std::cin, test_number_str );
    }
    int test_number = std::stoi( test_number_str );

    // Testing
    node.startTesting( test_number );

    ros::spin();
    return 0;
}
