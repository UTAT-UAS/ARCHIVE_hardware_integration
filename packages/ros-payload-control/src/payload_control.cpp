#include "payload_control.hpp"

PayloadControl::PayloadControl(ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150> &nh)
 : nh_(nh), servoVelocityPub_("/pld/servo_sent", &servoVelMsg_), 
 encoderRawPub_("/pld/encoder_raw", &encoderRawFbMsg_), 
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_)
{
    // hardware setup
    nh_.getHardware()->setBaud(57600);
    nh_.initNode();

    // servo setup
    PayloadControl::servoAttach();
    PayloadControl::servoWrite(0.0);

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
}

void PayloadControl::SetpointCb(const std_msgs::Float32 &msg)
{
    setpointMsg_ = msg;
}

void PayloadControl::PublishServoCommand()
{
    // ros
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
    if (servoOutput_ >= maxSpd_) {servoOutput_ = maxSpd_;}
    else if (servoOutput_ <= -maxSpd_) {servoOutput_ = -maxSpd_;}

    PayloadControl::servoWrite(servoOutput_);
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

void PayloadControl::servoAttach()
{ 
    pinMode(servoPin_, OUTPUT);

    // this setup sets the pin timer to 50 Hz and is only valid for 16 MHz clock
    TCCR1A = 0;  // Clear Timer1 control register A
    TCCR1B = 0;  // Clear Timer1 control register B
    TCNT1 = 0;   // Initialize counter

    ICR1 = 39999;  // Set TOP for 50 Hz (16 MHz / (8 * (39999 + 1))) 
    TCCR1A = (1 << WGM11);           // Fast PWM, mode 14 (ICR1 is TOP)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    TCCR1A |= (1 << COM1A1);         // Non-inverting mode on pin 9
}

void PayloadControl::servoWrite(float rps)
{
    int us = 1000 + ((rps - (-maxSpd_)) * (2000 - 1000)) / (maxSpd_*2);
	if (us < 1000) us = 1000;
	if (us > 2000) us = 2000;

	uint32_t dutyCycle = (uint32_t) (us * 2); // converts microseconds to timer ticks (ICR1 = 400000 -> 20 ms / .5 us ticks)
	OCR1A = dutyCycle;
}

void PayloadControl::Update()
{
    ControlLoop();
    PublishServoCommand();
    PublishEncoderFb();
    nh_.spinOnce();
}