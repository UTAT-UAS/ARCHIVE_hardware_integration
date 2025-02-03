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
    servoVelocityPub_.publish(&servoVelMsg_); 
}

void PayloadControl::PublishEncoderFb()
{
    noInterrupts();
    encoderRawFbMsg_.data = encoderRaw_;
    encoderLenFbMsg_.data = encoderLen_;
    interrupts();

    encoderRawPub_.publish(&encoderRawFbMsg_);
    encoderLenPub_.publish(&encoderLenFbMsg_);
}

void PayloadControl::PublishOperationState()
{
    // put op code into first index, state into second (int32multiarray)
    stateMsg_.data_length = 2;
    stateMsg_.data[0] = static_cast<int>(operation_);
    stateMsg_.data[1] = static_cast<int>(state_);

    operationDoneMsg_.data = operationDone_;

    stateMsgPub_.publish(&stateMsg_);
    operationDonePub_.publish(&operationDoneMsg_);
}