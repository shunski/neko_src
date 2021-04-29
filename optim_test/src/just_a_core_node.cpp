// just_a_core.cpp
#include <ros/ros.h>
#include <node_lib/Node.h>
#include <support_lib/Utilities.h>

class JustACore : public node::HeartratePublisherNode, public node::RegistrarNode
{};

int main(int argc, char **argv) {
    ros::init(argc, argv, "core_node");
    JustACore node;
    ros::spin();
    return 0;
}
