// McNode.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class RhMcNode : protected Node::MotionControllerNode
{
    public:
        McNodeRH()
            MotionControllerNode(RHLEG, "rhLegCommand", "rhCurrentState", "rhProcessedInfo", "rhLocomotionAction")
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhMcNode");
    RhMcNode node();
    ros::spin();
    return 0;
}
