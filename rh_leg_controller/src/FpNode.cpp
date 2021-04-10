// FpNode.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class RhFpNode : protected Node::FeedbackProcessorNode
{
    public:
        FeedbackProcessorNode()
            MotionControllerNode( RHLEG, "rhLegCommand", "rhProcessedFeedback", "rhTeensyFeedback", "rhHeartrateSubscriber" )
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhFpNode");
    RhFpNode node();
    ros::spin();
    return 0;
}
