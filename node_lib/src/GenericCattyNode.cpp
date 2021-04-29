#include <node_lib/Node.h>

using namespace node;

GenericCattyNode::GenericCattyNode( const std::string & NodeName ):
    nodeName( NodeName )
{
    support_srvs::RegistrationSrv registrationSrv;
    registrationSrv.request.nodeName = nodeName;
    ros::ServiceClient nodeRegistrationClient = this->serviceClient<support_srvs::RegistrationSrv>( nodeRegistrationTopicName );

    initSucceeded = nodeRegistrationClient.call( registrationSrv );

    int fail_limit = 150;
    while ( !initSucceeded ){
        if( fail_limit == 0 ){
            initSucceeded = false;
            break;
        }
        ROS_INFO("ERROR: Failed to initialize the node [%s].", nodeName.c_str());
        ros::Duration( 0.2 ).sleep();
        initSucceeded = nodeRegistrationClient.call( registrationSrv );
        fail_limit--;
    }
    if ( initSucceeded ){
        ROS_INFO("[%s]: Initialization Succeed.", nodeName.c_str());
        heartSoundPublisher = this->advertise<std_msgs::String>( registrationSrv.response.topicName, default_queue_size );
    } else {
        ROS_INFO("[%s]: Initialization Failed.", nodeName.c_str());
    }
}


void GenericCattyNode::publish_heartSound() const {
    std_msgs::String msg;
    msg.data = nodeName;
    heartSoundPublisher.publish( msg );
}


bool GenericCattyNode::didInitSucceed() const { return initSucceeded; }
