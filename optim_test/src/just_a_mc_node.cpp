// just_a_mc_node.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class JustAMcNode : public node::MotionControllerNode
{
    public:
        JustAMcNode():
            MotionControllerNode( RHLEG, "SomePart" )
        {}
};

int main( int argc, char **argv ) {
    ros::init( argc, argv, "mc_node" );
    JustAMcNode node;
    if ( !node.didInitSucceed()) return 1;
    ros::spin();
    return 0;
}
