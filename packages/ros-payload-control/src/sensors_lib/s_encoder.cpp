#include "payload_control.hpp"

void IRAM_ATTR PayloadControl::EncoderISR()
{
    // if (instance_) {
    //     instance_->ReadEncoder();
    // }
}

void PayloadControl::EncoderSetup()
{
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
}

void PayloadControl::ReadEncoder()
{
    lastEncoderLen_ = encoderLen_;
    unsigned char result = encoder_.process();
    if (result == DIR_CW) {
        encoderRaw_++;
    } else if (result == DIR_CCW) {
        encoderRaw_--;
    }
    encoderLen_ = ROT2LIN * encoderRaw_; // convert to length

    if (encoderLen_ != lastEncoderLen_) {
        lastEncoderChangeTime_ = millis();  // track the last movement time
    }
}