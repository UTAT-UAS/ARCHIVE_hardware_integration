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
    PayloadControl::contServoAttach();
    PayloadControl::contServoWrite(0.0);
    hookServo_.attach(hookServoPin_);
    
    attachInterrupt(digitalPinToInterrupt(pinA_), PayloadControl::EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB_), PayloadControl::EncoderISR, CHANGE);

    // subs
    ros::Subscriber<std_msgs::Float32>setpointSub_("/pld/servo_command", PayloadControl::SetpointCb);
    ros::Subscriber<std_msgs::Float32>hookSub_("/pld/hook_command", PayloadControl::SetpointCb);
    nh_.subscribe(setpointSub_);
    nh_.subscribe(hookSub_);

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

void PayloadControl::HookCb(const std_msgs::Float32 &msg)
{
    hookMsg_ = msg;
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

void PayloadControl::ControlLoop()
{
    // pi controller
    servoSetpoint_ = setpointMsg_.data;
    noInterrupts();
    encoderLen_ = conversion_ * encoderRaw_; // convert to length
    interrupts();

    curError_ = servoSetpoint_ - encoderLen_;  // length error
    //float derivative = (curError_ - lastError_) / dt_;
    servoOutput_ = kp_ * curError_ + ki_ * integral_;

    // reset int and output if error is small
    if (abs(curError_) < 0.01) {
        servoOutput_ = 0;
        integral_ = 0;
    }    

    // clamping
    if (servoOutput_ >= maxSpd_) {servoOutput_ = maxSpd_;}
    else if (servoOutput_ <= -maxSpd_) {servoOutput_ = -maxSpd_;}
    else {integral_ += curError_ * dt_;}
    lastError_ = curError_;
    constrain(hookMsg_.data, 0, 1);

    // write to servos
    hookServo_.write(hookMsg_.data * 90);
    PayloadControl::contServoWrite(servoOutput_);
}

void PayloadControl::EncoderISR()
{
    if (instance_) {
        instance_->HandleEncoder();
    }
}

void PayloadControl::HandleEncoder()
{
    unsigned char result = encoder_.process();
    if (result == DIR_CW) {
        encoderRaw_++;
    } else if (result == DIR_CCW) {
        encoderRaw_--;
    }
}

void PayloadControl::contServoAttach()
{ 
    pinMode(contServoPin_, OUTPUT);

    // this setup sets the pin timer to 50 Hz and is only valid for 16 MHz clock
    TCCR1A = 0;  // Clear Timer1 control register A
    TCCR1B = 0;  // Clear Timer1 control register B
    TCNT1 = 0;   // Initialize counter

    ICR1 = 39999;  // Set TOP for 50 Hz (16 MHz / (8 * (39999 + 1))) 
    TCCR1A = (1 << WGM11);           // Fast PWM, mode 14 (ICR1 is TOP)
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8
    TCCR1A |= (1 << COM1A1);         // Non-inverting mode on pin 9
}

void PayloadControl::contServoWrite(float rps)
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