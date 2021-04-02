// MotionController.cpp implements MotionController class in Body.h
#include <jcatty_body_lib/Body.h>

MotionController::MotionController ( Part part, jcatty_body_msgs::PartCmdMsg::ConstPtr & msg )
{
    this->set ( part, msg );
}

void MotionController::set ( Part part, jcatty_body_msgs::PartCmdMsg::ConstPtr & msg )
{
    this->set_sequence ( msg );
}

void MotionController::set_sequence ( jcatty_body_msgs::PartCmdMsg::ConstPtr & msg ) {
    for (std::vector<Part>::const_iterator it = msg->command.begin(); msg->command.end(); ++it){
        std::vector<KondoServo> servoSet;
        if (it->servoCommand1 != NULL){
            servoSet(KondoServo(it->servoCommand1));
        }
        if (it->servoCommand2 != NULL){
            servoSet(KondoServo(it->servoCommand2));
        }
        if (it->servoCommand3 != NULL){
            servoSet(KondoServo(it->servoCommand3));
        }

        std::vector<BrushedMotor> brushedMotorSet;
        if (it->brushedMotorCommand1 != NULL){
            brushedMotorSet(BrushedMotor(it->brushedMotorCommand1));
        }
        if (it->brushedMotorCommand2 != NULL){
            brushedMotorSet(BrushedMotor(it->brushedMotorCommand2));
        }

        std::vector<BrushlessMotor> brushlessMotorSet;
        if (it->brushlessMotorCommand != NULL){
            brushlessMotorSetSet(brushlessMotorCommand(it->brushedCommand));
        }

        expectedStates.push_back( Part( servoSet, brushedMotorSet, brushlessMotorSet ));
    }
}

void MotionController::initializePosition(){
    expectedStates.front();
}

void MotionController::procced() {
    if (currentScene == expectedStates.begin()){ //first case
        ros::Time currentTime = ros::Time::now();
        actualSceneDuration = currentTime - timeOfActionStart;
        timeOfLastAction = currentTime;
    }
    else{
        ros::Time currentTime = ros::Time::now();
        actualSceneDuration = currentTime - timeOfLastAction;
        timeOfLastAction = currentTime;
    }
    ++currentScene;
}

void MotionController::set_CommandMsg( jcattty_teensy_msgs::HindlegCommandMsg & msg ){
    currentState->set_CommandMsg(msg);
}

jcatty_locomotion_action::MotionControllFeedback MotionController::get_feedbackMsg() const {
    jcatty_locomotion_action::MotionControllFeedback feedback;
    feedback.currentState = currentState->get_PartMsg();
    feedback.currentSceneDuration = actualSceneDuration;
    return feedback;
}

jcatty_locomotion_action::MotionControllResult MotionController::get_resultMsg() const {
    jcatty_locomotion_action::MotionControllResult result;
    result.totalDuration = ros::Time::now() - timeOfActionStart;
    result.finalState = * currentState;
    return result;
}
