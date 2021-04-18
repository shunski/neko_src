// McNode.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class RhMcNode : protected Node::MotionControllerNode
{
    public:
        RhMcNode()
            MotionControllerNode( RHLEG, "rhLeg" )
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhMcNode");
    RhMcNode node();
    if ( !RhNode.isValid()) return 0;
    ros::spin();
    return 0;
}
