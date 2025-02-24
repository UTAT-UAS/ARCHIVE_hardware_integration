#include "payload_control.hpp"

void PayloadControl::forceSensorSetup() {
  // put your setup code here, to run once:
  pinMode(forceAnalogPin_, INPUT);
}

void PayloadControl::forceRead() {
  // put your main code here, to run repeatedly:
  rawForce_ = 294.3 - analogRead(forceAnalogPin_) * (294.3 / 4095);
  force_ = alpha_ * rawForce_ + (1 - alpha_) * force_;
}