#include "payload_control.hpp"

PayloadControl::PayloadControl(ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150> &nh)
 : nh_(nh), servoVelocityPub_("/pld/servo_sent", &servoVelMsg_), 
 encoderRawPub_("/pld/encoder_raw", &encoderRawFbMsg_), 
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_)
{
    // hardware setup
    nh_.getHardware()->setBaud(57600);
    nh_.initNode();
    pldServo_.attach(6);
    pldServo_.writeMicroseconds(2000);

    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
    attachInterrupt(digitalPinToInterrupt(pinB_), PayloadControl::EncoderISR, CHANGE);

    // subs
    ros::Subscriber<std_msgs::Float32>setpointSub_("/pld/servo_command", PayloadControl::SetpointCb);
    nh_.subscribe(setpointSub_);

    // pubs
    nh_.advertise(servoVelocityPub_);
    nh_.advertise(encoderRawPub_);
    nh_.advertise(encoderLenPub_);

    instance_ = this;
}

PayloadControl::~PayloadControl()
{
    pldServo_.detach();
}

void PayloadControl::SetpointCb(const std_msgs::Float32 &msg)
{
    setpointMsg_ = msg;
}

void PayloadControl::PublishServoCommand()
{
    servoVelMsg_.data = servoOutput_;
    servoVelocityPub_.publish(&servoVelMsg_); 
}

void PayloadControl::PublishEncoderFb()
{
    encoderRawFbMsg_.data = encoderRaw_;
    encoderLenFbMsg_.data = encoderLen_;

    encoderRawPub_.publish(&encoderRawFbMsg_);
    encoderLenPub_.publish(&encoderLenFbMsg_);
}

void PayloadControl::ControlLoop()
{
    // pd controller
    servoSetpoint_ = setpointMsg_.data;
    encoderLen_ = conversion_ * encoderRaw_; // convert to length

    curError_ = servoSetpoint_ - encoderLen_;  // length error
    servoOutput_ = kp_ * curError_;

    // clamping
    if (servoOutput_ >= max_) {servoOutput_ = max_;}
    else if (servoOutput_ <= 0) {servoOutput_ = 0;}

    pldServo_.write(servoOutput_);
}

void PayloadControl::EncoderISR()
{
    if (instance_) {
        instance_->HandleEncoder();
    }
}

void PayloadControl::HandleEncoder()
{
    int32_t currentA = digitalRead(pinA_);
    int32_t currentB = digitalRead(pinB_);
    if (currentA != lastA_) {
        if (currentA == currentB) {
            encoderRaw_++;  // Clockwise
        } else {
            encoderRaw_--;  // Counter-clockwise
        }
    }
    lastA_ = currentA;
}

void PayloadControl::Update()
{
    ControlLoop();
    PublishServoCommand();
    PublishEncoderFb();
    nh_.spinOnce();
}