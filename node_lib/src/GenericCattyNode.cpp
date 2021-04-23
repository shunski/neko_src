#include <node_lib/Node.h>

using namespace Node;

GenericCattyNode::GenericCattyNode( const std::string NodeName ):
    nodeName( NodeName )
{
    support_srvs::InitRegistrationService initSrv;
    initSrv->node_name = nodeName;
    ros::ServiceClient nodeRegistrationClient = this->serviceClient<support_srvs::InitRegistrationService>( nodeRegistrationTopicName );

    init_succeeded = nodeRegistrationClient.call();

    int fail_limit = 20;
    while ( !init_succeeded ){
        if( fail_limit == 0 ){
            init_succeeded = false;
            break;
        }
        ROS_INFO("ERROR: Failed to initialize the node [%s].", nodeName.c_str());
        fail_limit--;
    }
}
bool GenericCattyNode::didInitSucceed() const { return init_succeeded; }
