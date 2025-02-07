#include "payload_control.hpp"

void PayloadControl::forceSensorSetup() {
  // put your setup code here, to run once:
  pinMode(pinForce_, INPUT);
}

void PayloadControl::forceRead() {
  // put your main code here, to run repeatedly:
  force_ = analogRead(pinForce_) * (294.3/1023.);
}