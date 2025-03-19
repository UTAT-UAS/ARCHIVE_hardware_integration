#include "payload_control.hpp"

#define DIR_NONE 0x00           // No complete step yet.
#define DIR_CW   0x10           // Clockwise step.
#define DIR_CCW  0x20           // Anti-clockwise step.
#define R_START     0x3
#define R_CW_BEGIN  0x1
#define R_CW_NEXT   0x0
#define R_CW_FINAL  0x2
#define R_CCW_BEGIN 0x6
#define R_CCW_NEXT  0x4
#define R_CCW_FINAL 0x5

const unsigned char ttable[8][4] = {
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},                // R_CW_NEXT
    {R_CW_NEXT,  R_CW_BEGIN,  R_CW_BEGIN,  R_START},                // R_CW_BEGIN
    {R_CW_NEXT,  R_CW_FINAL,  R_CW_FINAL,  R_START | DIR_CW},       // R_CW_FINAL
    {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},                // R_START
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},                // R_CCW_NEXT
    {R_CCW_NEXT, R_CCW_FINAL, R_CCW_FINAL, R_START | DIR_CCW},      // R_CCW_FINAL
    {R_CCW_NEXT, R_CCW_BEGIN, R_CCW_BEGIN, R_START},                // R_CCW_BEGIN
    {R_START,    R_START,     R_START,     R_START}                 // ILLEGAL
};

void PayloadControl::EncoderSetup()
{
    pinMode(pinA_, INPUT_PULLUP);
    pinMode(pinB_, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA_), instance_->EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB_), instance_->EncoderISR, CHANGE);
}   

void IRAM_ATTR PayloadControl::EncoderISR()
{
    instance_->ReadEncoder();
}

void IRAM_ATTR PayloadControl::ReadEncoder()
{
    const unsigned char state = (digitalRead(pinA_) << 1) | digitalRead(pinB_);
    encState_ = ttable[encState_ & 0x07][state];
    
    if (encState_ & DIR_CW) {
        encoderRawISR_++;
    } else if (encState_ & DIR_CCW) {
        encoderRawISR_--;
    }
    
    encoderLenISR_ = ROT2LIN * encoderRawISR_;
}

void PayloadControl::ProcessEncoder()
{
    noInterrupts();
    lastEncoderLen_ = encoderLen_;
    encoderLen_ = encoderLenISR_;
    if (lastEncoderLen_ != encoderLenISR_) {
        lastEncoderChangeTime_ = millis();
    }
    interrupts();

    // filter velocity
    float rawVelocity = (encoderLen_ - lastEncoderLen_) / dt_;
    encoderVel_ = alphaVelEnc_ * rawVelocity + (1 - alphaVelEnc_) * encoderVel_;
    if (fabs(encoderVel_) < 0.0001) {
        encoderVel_ = 0;
    }
}