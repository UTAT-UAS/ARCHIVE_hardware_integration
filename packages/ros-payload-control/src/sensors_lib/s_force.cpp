#include "payload_control.hpp"

void PayloadControl::forceSensorSetup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT);
}

void PayloadControl::forceRead() {
  // put your main code here, to run repeatedly:
  rawForce_ = 294.3 - analogRead(A0) * (294.3 / 1023.);
  force_ = alpha_ * rawForce_ + (1 - alpha_) * force_;
}