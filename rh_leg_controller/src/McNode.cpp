// McNode.cpp
#include <ros/ros.h>
#include <jcatty_simple_struct/Node.h>

class RhMcNode : protected Node::MotionControllerNode
{
    public:
        McNodeRH()
            MotionControllerNode("rhLegCommand", "rhCurrentState", "rhProcessedInfo", "rhLocomotionAction")
        {}
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RhMcNode");
    RhMcNode node();
    ros::spin();
    return 0;
}
