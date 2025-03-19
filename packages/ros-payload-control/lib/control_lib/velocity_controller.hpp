#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

#include <iostream>
#include "pid.hpp"  // custom PID library

class VelocityPID
{
public:
    VelocityPID(float dt, float max, float min, float Kp, float Ki, float Kd, float integ_clamp, float alpha, float tau, int dir)
    : velZ_(dt, max, min, Kp, Ki, Kd, integ_clamp, alpha, tau), direction_(dir)
    {}

    // calc (calc stands for calculate if you just joined the stream) the error and return pid computed values in vector3
    float compute(float velSetpoint, float currentVel) {
        return 1500 + direction_ * velZ_.run(velSetpoint, currentVel);
    }
    PID velZ_;


private:
    int direction_{1};
};

#endif