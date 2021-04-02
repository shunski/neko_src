// FpNode.cpp
#include <ros/ros.h>
#include <jcatty_simple_struct/Node.h>

class RhFpNode : protected Node::FeedbackProcessorNode
{
    public:
        FeedbackProcessorNode()
            MotionControllerNode("rhLegCommand", "rhCurrentState", "rhProcessedInfo", "rhLocomotionAction")
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhFpNode");
    RhFpNode node();
    ros::spin();
    return 0;
}
