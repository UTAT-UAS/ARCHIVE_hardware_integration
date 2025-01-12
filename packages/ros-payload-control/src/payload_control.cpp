#include "payload_control.hpp"

PayloadControl::PayloadControl(ros::NodeHandle &nh) 
{
    nh_ = nh;
    servoCommandPub_ = nh_.advertise<std_msgs::Float32>("servo_command", 1);
    encoderSub_ = nh_.subscribe("encoder", 1, &PayloadControl::EncoderCallback, this);
}

void PayloadControl::PayloadUp()
{
       
}
void PayloadControl::PayloadHold()
{
    servoPosSetpoint_ = 0;
}

void PayloadControl::PayloadDown()
{
    servoPosSetpoint_ = 1;
}

void PayloadControl::Control()
{
    // pid control
    curError_ = servoPosSetpoint_ - encoderVal_;
    servoVelOutput_ = kp_ * curError_;
}

void PayloadControl::PublishServoCommand()
{
    servoVelMsg_.data = servoVelOutput;
    servoCommandPub_.publish(servoVelMsg_); 
}

void PayloadControl::EncoderCallback(std_msgs::Float32 msg)
{
    encoderSub_.subscribe()

}