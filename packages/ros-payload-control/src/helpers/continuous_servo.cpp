#include "payload_control.hpp"

void PayloadControl::contServoAttach()
{
    contServo_.attach(contServoPin_); // ✅ Attach the servo to the ESP32 PWM pin
}

void PayloadControl::contServoWrite(float rps)
{
    // ✅ Convert speed (rps) to microseconds (1000-2000 µs range)
    int us = 1500 + (rps * (500.0 / maxSpd_));  

    // ✅ Clamp values to valid servo range (1000-2000 µs)
    us = constrain(us, 1000, 2000);

    // ✅ Write to the continuous rotation servo
    contServo_.writeMicroseconds(us);
}
