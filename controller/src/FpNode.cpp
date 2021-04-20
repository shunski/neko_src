// FpNode.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class RhFpNode : public Node::FeedbackProcessorNode
{
    public:
        RhFpNode():
            FeedbackProcessorNode( RHLEG, "rhLeg")
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhFpNode");
    RhFpNode node;
	if ( !node.isValid()) return 1;
    ros::spin();
    return 0;
}
