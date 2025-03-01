#include "payload_control.hpp"

PayloadControl::PayloadControl() 
 :
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_),
 encoderVelPub_("/pld/encoder_vel", &encoderVelMsg_),
 stateMsgPub_("/pld/state_fb", &stateMsg_),
 operationDonePub_("/pld/op_done", &operationDoneMsg_),
 forcePub_("/pld/force", &forceMsg_),
 waterlevelPub_("/pld/water", &waterlevelMsg_)
{
    instance_ = this;
}

void PayloadControl::SensorsSetup()
{
    // servo setup
    contServoAttach();
    contServoWrite(0.0);
    hookServo_.attach(hookServoPin_);
    hookServo_.write(2000);

    EncoderSetup();
    ForceSensorSetup();
    //TofSensorSetup();
}

void PayloadControl::PositionControlLoop(float lenSetpoint, float hookSetpoint)
{
    // pi controller
    curError_ = lenSetpoint - encoderLen_;  // length error
    //float derivative = (curError_ - lastError_) / dt_;
    servoOutput_ = kp_ * curError_ + ki_ * integral_; 

    // clamping
    if (servoOutput_ >= maxSpd_) {servoOutput_ = maxSpd_;}
    else if (servoOutput_ <= -maxSpd_) {servoOutput_ = -maxSpd_;}
    else {integral_ += curError_ * dt_;}
    lastError_ = curError_;
    hookSetpoint = constrain(hookSetpoint, 0, 1);

    // if error small reset
    if (abs(curError_) < 0.005) {
        servoOutput_ = 0;
        integral_ = 0;
    }

    // write to servos
    hookServo_.writeMicroseconds(map(hookSetpoint, 0, 1, 1025, 1875));
    contServoWrite(-servoOutput_);
}

void PayloadControl::VelocityControlLoop(float rps)
{
    // i controller
    curVelError_ = rps - (filteredVel_ / R);  // velocity error
    velIntegral_ += curVelError_ * dt_;
    contServoWrite(-(rps + kiVel_ * velIntegral_));
}

void PayloadControl::ReadSensors()
{
    // process encoder
    ProcessEncoder();
    // read force sensor
    ForceRead();
}

void PayloadControl::SwitchState(State state)
{
    state_ = state;
}

void PayloadControl::UpdatePayload()
{
    // update the hold position for continuous servo
    if (operation_ != OPCODE::STOPPED) {
        stoppedEncoderLen_ = encoderLen_;
    }
    // op code & state machine
    switch (operation_)
    {
        case OPCODE::STOPPED:
            PositionControlLoop(stoppedEncoderLen_, 1);
            SwitchState(State::IDLE);
            operationDone_ = true;
            break;

        case OPCODE::PICKUP:
            operationDone_ = false;
            switch (state_)
            {
                case State::IDLE:

                    contServoWrite(0);
                    PositionControlLoop(encoderLen_, 0);
                    SwitchState(State::UNSPOOL);    
                    waitTimerStart_ = millis();
                    break;

                case State::UNSPOOL:

                    // if the encoder has reached the desired length with tolerances
                    // wait for hook dynamics
                    if (millis() - waitTimerStart_ <= 1000) {
                        break;
                    }
                    PositionControlLoop(pickupLen_, 0);
                    if (abs(pickupLen_ - encoderLen_) < 0.1) {
                        waitTimerStart_ = millis();
                        SwitchState(State::WAIT);
                    }
                    break;

                case State::WAIT:
                    PositionControlLoop(pickupLen_, 0);
                    if (millis() - waitTimerStart_ >= pickupTime_ * 1000) {
                        SwitchState(State::RESPOOL);
                    }
                    break;

                case State::RESPOOL:

                    if (encoderLen_ > 0.1) {
                        PositionControlLoop(0.05, 0);  // go back to 0 height with encoder fb
                        waitTimerStart_ = millis();
                    }
                    else {
                        VelocityControlLoop(-1.5);  // slowly retract

                        // if force or encoder has not changed for a certain time then stop
                        if (force_ >= 10 || (millis() - lastEncoderChangeTime_) > 700) { 
                            encoderRawISR_ = 0;
                            encoderLen_ = 0;
                            stoppedEncoderLen_ = 0;
                            operation_ = OPCODE::STOPPED;
                        }
                    } 
                    break;
            }
            break;

        case OPCODE::DISPENSE:
            operationDone_ = false;
            switch (state_)
            {
                case State::IDLE:
                    // enable hook servo
                    PositionControlLoop(encoderLen_, 1);
                    SwitchState(State::UNSPOOL);    
                    waitTimerStart_ = millis();
                    
                    break;
                case State::UNSPOOL:
                    // wait for hook dynamics
                    if (millis() - waitTimerStart_ <= 1000) {
                        // store last water amount
                        break;
                    }
                    PositionControlLoop(dispenseLen_, 1);
                    if (abs(dispenseLen_ - encoderLen_) < 0.2) {
                        waitTimerStart_ = millis();
                        SwitchState(State::WAIT);
                        
                    }
                    break;
                case State::WAIT:
                    PositionControlLoop(dispenseLen_, 1);
                    // logic here to check water levels
                    // if (current water amount - last water amount > 600) {
                    if (millis() - waitTimerStart_ >= 5 * 1000) {
                        SwitchState(State::RESPOOL);
                        contServoWrite(1);  // slowly retract
                    }
                    break;
                case State::RESPOOL:
                    // slowly retract and wait for force sensor
                    contServoWrite(1);  // slowly retract at 1.5 rad/s upwards
                    if (force_ >= 10) { // if force is detected, stop
                        encoderRawISR_ = 0;
                        encoderLen_ = 0;
                        stoppedEncoderLen_ = 0;
                        operation_ = OPCODE::STOPPED;
                    }
                    break;
            }
            break;

        case OPCODE::RESET:
            operationDone_ = false;
            state_ = State::RESPOOL;
            VelocityControlLoop(-1);  // slowly retract
            hookServo_.writeMicroseconds(1100);
            
            // state transition condition
            if (force_ >= 10) { // if force is detected, stop
                encoderRawISR_ = 0;
                encoderLen_ = 0;
                stoppedEncoderLen_ = 0;
                operation_ = OPCODE::STOPPED;
            }
            break;

        case OPCODE::MANUAL:
            operationDone_ = false;
            PositionControlLoop(manualServoSetpoint_, 0);
            break;
        
        default:
            break;
    }
    ReadSensors();
    PublishServoCommand();
    PublishSensorsFb();
    PublishOperationState();
}