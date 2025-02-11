#include "payload_control.hpp"

void PayloadControl::EncoderISR()
{
    if (instance_) {
        instance_->HandleEncoder();
    }
}

void PayloadControl::HandleEncoder()
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