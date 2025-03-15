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
  force1_ = alpha_ * rawForce1_ + (1 - alpha_) * force1_;
  force2_ = alpha_ * rawForce2_ + (1 - alpha_) * force2_;
}