// just_a_fp_node.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class JustAFpNode : public node::FeedbackProcessorNode
{
    public:
        JustAFpNode():
            FeedbackProcessorNode( RHLEG, "SomePart")
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "fp_node");
    JustAFpNode node;
	if ( !node.didInitSucceed()) return 1;
    ros::spin();
    return 0;
}
