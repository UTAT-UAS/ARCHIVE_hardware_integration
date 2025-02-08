#include "payload_control.hpp"

void PayloadControl::SetpointCb(const std_msgs::Float32 &msg)
{
    instance_->manualServoSetpoint_ = msg.data;
}

void PayloadControl::RecieveStateCb(const std_msgs::Int32 &msg)
{
    instance_->operation_ = static_cast<OPCODE>(msg.data);
}

void PayloadControl::PublishServoCommand()
{
    // ros
    servoVelMsg_.data = servoOutput_;
    //servoVelocityPub_.publish(&servoVelMsg_); 
}

void PayloadControl::PublishSensorsFb()
{
    noInterrupts();
    encoderLenFbMsg_.data = encoderLen_;
    interrupts();

    forceMsg_.data = force_;
    waterlevelMsg_.data = waterlevel_;

    encoderLenPub_.publish(&encoderLenFbMsg_);
    forcePub_.publish(&forceMsg_);
    waterlevelPub_.publish(&waterlevelMsg_);    
}

void PayloadControl::PublishOperationState()
{
    // publish state and if the operation is done
    stateMsg_.data = static_cast<int>(state_);
    operationDoneMsg_.data = operationDone_;

    stateMsgPub_.publish(&stateMsg_);
    operationDonePub_.publish(&operationDoneMsg_);
}


