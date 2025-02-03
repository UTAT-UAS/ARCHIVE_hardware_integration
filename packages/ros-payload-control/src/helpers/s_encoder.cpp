#include "payload_control.hpp"

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
    noInterrupts();
    encoderLen_ = conversion_ * encoderRaw_; // convert to length
    interrupts();
}