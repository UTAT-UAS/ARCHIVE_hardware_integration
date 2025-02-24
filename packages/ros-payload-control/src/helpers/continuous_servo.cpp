#include "payload_control.hpp"

void PayloadControl::contServoAttach()
{
    contServo_.attach(contServoPin_);
}

void PayloadControl::contServoWrite(float rps)
{
    int us = 1000 + ((rps - (-maxSpd_)) * (2000 - 1000)) / (maxSpd_*2); 
    us = constrain(us, 1000, 2000);
    contServo_.writeMicroseconds(us);
}
