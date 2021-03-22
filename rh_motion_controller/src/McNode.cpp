/* McNode.cpp implementes McNode.h
 * private member: ros::NodeHandle nh;
 *                 MotionController mc;
 *
 *                 actionlib::SimpleActionServer<jcatty_action::MotionControll> core_server;
 *                 jcatty_action::MotionControll result;
 *                 jcatty_action::MotionControll feedback;
 *
 *                 ros::Publisher rf_publisher;
 *                 ros::Publisher lf_publisher;
 *                 ros::Publisher rh_publisher;
 *                 ros::Publisher lh_publisher;
 *
 *                 jcatty_msgs::rfInfo rf_info;
 *                 jcatty_msgs::lfInfo lf_info;
 *                 jcatty_msgs::rhInfo rh_info;
 *                 jcatty_msgs::lhInfo lh_info;
 *
 */


rf_callback( const jcatty_action::rfInfo::ConstPtr& msg ){
    msg->servo_deg;
    msg->servo_temp;
    msg->servo_current;
}
lf_callback( const jcatty_action::lfInfo::ConstPtr& msg ){ lf_info = *msg; }
rh_callback( const jcatty_action::rhInfo::ConstPtr& msg ){ rh_info = *msg; }
lh_callback( const jcatty_action::lhInfo::ConstPtr& msg ){ lh_info = *msg; }


McNode::core_callback( const jcatty_action::MotionControllGoalConstPtr &goal ){
     bool success = true;

     if(success) {
         result = feedback;
         ROS_INFO("MotioncontrollAction Succeeded");
         core_server.setSucceeded(result);
     }
}
