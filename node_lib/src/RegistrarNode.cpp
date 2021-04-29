#include <node_lib/Node.h>

using namespace node;

RegistrarNode::RegistrarNode(){
    registrationServer = this->advertiseService( nodeRegistrationTopicName, &node::RegistrarNode::registrationCallback, this );
}

void RegistrarNode::disregister_disqualifiedNode() {
    for( auto it = registeredNodeList.begin(); it != registeredNodeList.end(); ++it )
    {
        ros::Duration durationFromLastRefreshed = ros::Time::now() - std::get<1>( it->second );
        if ( durationFromLastRefreshed > heartrate*2 )
        {
            registeredNodeList.erase( it );
            ROS_INFO("[%s] is disqualified and removed from the registeredNodeList.", it->first.c_str());
        }
    }
}


std::vector<std::string> RegistrarNode::get_allLiveNodes(){
    std::vector<std::string> liveNodeList;
    for( auto it = registeredNodeList.begin(); it != registeredNodeList.end(); ++it ){
        liveNodeList.push_back( it->first );
    }
    return liveNodeList;
}


bool RegistrarNode::isRegistered( const std::string & NodeName ){
    for ( auto it = registeredNodeList.begin(); it!=registeredNodeList.end(); ++it ){
        if ( it->first == NodeName){
            return true;
        }
    }
    return false;
}


bool RegistrarNode::registrationCallback( support_srvs::RegistrationSrv::Request & req, support_srvs::RegistrationSrv::Response & res ){
    std::string newTopicName;
    do{
        newTopicName = createRandomStringOfSize( rand() % 10 + 10 );
    } while ( isRegistered( newTopicName ));

    ros::Subscriber newSub = this->subscribe( newTopicName, 10, &node::RegistrarNode::heartrateSoundSubscriberCallback, this );
    registeredNodeList[ req.nodeName ] = std::make_tuple( newSub, ros::Time::now());
    return true;
}


void RegistrarNode::heartrateSoundSubscriberCallback( const std_msgs::String::ConstPtr & msg ) {
     std::get<1>( registeredNodeList[ msg->data ] ) = ros::Time::now();
}

void RegistrarNode::heartPumped(){
    ( heartrate*0.5 ).sleep();
    disregister_disqualifiedNode();
}
