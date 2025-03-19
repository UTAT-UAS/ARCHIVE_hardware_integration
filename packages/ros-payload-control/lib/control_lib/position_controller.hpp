#ifndef POSITION_CONTROLLER_HPP
#define POSITION_CONTROLLER_HPP

#include <iostream>
#include "pid.hpp"  // custom PID library

class PositionPID
{
public:
    PositionPID(float dt, float max, float min, float Kp, float Ki, float Kd, float integ_clamp, float alpha, float tau)
    : posZ_(dt, max, min, Kp, Ki, Kd, integ_clamp, alpha, tau)
    {}

    // calc (calc stands for calculate if you just joined the stream) the error and return pid computed values in vector3
    float compute(float posSetpoint, float currentPos) {
        return posZ_.run(posSetpoint, currentPos);
    }

private:
    PID posZ_;
};

#endif