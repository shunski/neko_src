// just_a_core.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class just_a_core : public Node::heartratePublisherNode
{
    public:
        RhFpNode():
            FeedbackProcessorNode( RHLEG, "rhLeg" )
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhFpNode");
    RhFpNode node;
	if ( !node.isValid()) return 1;
    ros::spin();
    return 0;
}
