#include "payload_control.hpp"

void PayloadControl::ForceSensorSetup() {
  // put your setup code here, to run once:
  pinMode(force1AnalogPin_, INPUT);
  pinMode(force2AnalogPin_, INPUT);
  
}

void PayloadControl::ForceRead() {
  // put your main code here, to run repeatedly:
  rawForce1_ = 294.3 - analogRead(force1AnalogPin_) * (294.3 / 4095.0);
  rawForce2_ = 294.3 - analogRead(force2AnalogPin_) * (294.3 / 4095.0);

  if (rawForce1_ > 294 || rawForce2_ > 294) {
    rawForce1_ = 0;
    rawForce2_ = 0;
    return;
  }
  
  force1_ = alphaForce_ * rawForce1_ + (1 - alphaForce_) * force1_;
  force2_ = alphaForce_ * rawForce2_ + (1 - alphaForce_) * force2_;
}