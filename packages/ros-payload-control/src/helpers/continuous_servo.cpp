#include "payload_control.hpp"

void PayloadControl::contServoAttach()
{
    contServo_.attach(contServoPin_);
}

void PayloadControl::contServoWrite(float rps)
{
    int us = 1500 + (rps * (500.0 / maxSpd_));  
    us = constrain(us, 1000, 2000);
    contServo_.writeMicroseconds(us);
}
