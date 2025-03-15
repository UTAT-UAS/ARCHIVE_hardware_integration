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
    //servoVelMsg_.data = servoOutput_;
    //servoVelocityPub_.publish(&servoVelMsg_); 
}

void PayloadControl::PublishSensorsFb()
{
    encoderLenFbMsg_.data = encoderLen_;
    encoderVelMsg_.data = filteredVel_;
    forceMsg_.header.stamp = nh_.now();
    forceMsg_.point.x = force1_;
    forceMsg_.point.y = force2_;
    weightMsg_.data = weight_;

    encoderLenPub_.publish(&encoderLenFbMsg_);
    encoderVelPub_.publish(&encoderVelMsg_);
    forcePub_.publish(&forceMsg_);
    weightPub_.publish(&weightMsg_);    
}

void PayloadControl::PublishOperationState()
{
    // publish state and if the operation is done
    stateMsg_.data = static_cast<int>(state_);
    operationDoneMsg_.data = operationDone_;

    stateMsgPub_.publish(&stateMsg_);
    operationDonePub_.publish(&operationDoneMsg_);
}


