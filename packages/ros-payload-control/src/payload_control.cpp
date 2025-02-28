#include "payload_control.hpp"

PayloadControl::PayloadControl() 
 :
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_),
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

void PayloadControl::ControlLoop(float lenSetpoint, float hookSetpoint)
{
    // pi controller

    curError_ = lenSetpoint - encoderLen_;  // length error
    //float derivative = (curError_ - lastError_) / dt_;
    servoOutput_ = kp_ * curError_ + ki_ * integral_;

    // reset int and output if error is small
    if (abs(curError_) < 0.01) {
        servoOutput_ = 0;
        integral_ = 0;
    }    

    // clamping
    if (servoOutput_ >= maxSpd_) {servoOutput_ = maxSpd_;}
    else if (servoOutput_ <= -maxSpd_) {servoOutput_ = -maxSpd_;}
    else {integral_ += curError_ * dt_;}
    lastError_ = curError_;
    hookSetpoint = constrain(hookSetpoint, 0, 1);

    // write to servos
    hookServo_.write(map(hookSetpoint, 0, 1, 1000, 2000));
    contServoWrite(-servoOutput_);
}

void PayloadControl::ReadSensors()
{
    // read force sensor
    ForceRead();
    // read tof sensor
    //TofRead();
}

void PayloadControl::SwitchState(State state)
{
    state_ = state;
}

void PayloadControl::UpdatePayload()
{
    // op code & state machine
    switch (operation_)
    {
        case OPCODE::STOPPED:

            PayloadControl::contServoWrite(0.0);
            SwitchState(State::IDLE);
            operationDone_ = true;
            break;

        case OPCODE::PICKUP:
            operationDone_ = false;
            switch (state_)
            {
                case State::IDLE:

                    contServoWrite(0);
                    ControlLoop(encoderLen_, 0);
                    SwitchState(State::UNSPOOL);    
                    waitTimerStart_ = millis();
                    break;

                case State::UNSPOOL:

                    // if the encoder has reached the desired length with tolerances
                    // wait for hook dynamics
                    if (millis() - waitTimerStart_ <= 1000) {
                        break;
                    }
                    ControlLoop(pickupLen_, 0);
                    if (abs(pickupLen_ - encoderLen_) < 0.1) {
                        waitTimerStart_ = millis();
                        SwitchState(State::WAIT);
                    }
                    break;

                case State::WAIT:

                    ControlLoop(pickupLen_, 0);
                    if (millis() - waitTimerStart_ >= pickupTime_ * 1000) {
                        SwitchState(State::RESPOOL);
                    }
                    break;

                case State::RESPOOL:

                    if (encoderLen_ > 0.1) {
                        ControlLoop(0.05, 0);  // go back to 0 height with encoder fb
                    }
                    else {
                        contServoWrite(1.3);  // slowly retract
                        if (force_ >= 10) { // if force is detected, stop
                            ControlLoop(0, 1);
                            operation_ = OPCODE::STOPPED;
                            operationDone_ = true;
                            encoderRaw_ = 0;
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
                    ControlLoop(encoderLen_, 1);
                    SwitchState(State::UNSPOOL);    
                    waitTimerStart_ = millis();
                    
                    break;
                case State::UNSPOOL:
                    // wait for hook dynamics
                    if (millis() - waitTimerStart_ <= 1000) {
                        // store last water amount
                        break;
                    }
                    ControlLoop(dispenseLen_, 1);
                    if (abs(dispenseLen_ - encoderLen_) < 0.2) {
                        waitTimerStart_ = millis();
                        SwitchState(State::WAIT);
                        
                    }
                    break;
                case State::WAIT:
                    ControlLoop(dispenseLen_, 1);
                    // logic here to check water levels
                    // if (current water amount - last water amount > 600) {
                    if (millis() - waitTimerStart_ >= 5 * 1000) {
                        SwitchState(State::RESPOOL);
                        contServoWrite(-1.5);  // slowly retract
                    }
                    break;
                case State::RESPOOL:
                    // slowly retract and wait for force sensor
                    contServoWrite(1.5);  // slowly retract
                    if (force_ >= 10) { // if force is detected, stop
                        ControlLoop(0, 1);
                        operation_ = OPCODE::STOPPED;
                        operationDone_ = true;
                        encoderRaw_ = 0;
                    }
                    break;
            }
            break;

        case OPCODE::RESET:
            state_ = State::RESPOOL;
            contServoWrite(2);  // slowly retract
            hookServo_.write(1000);
            // state transition condition
            if (force_ >= 10) { // if force is detected, stop
                encoderRaw_ = 0;
                ControlLoop(0, 1);
                operation_ = OPCODE::STOPPED;
                operationDone_ = true;
            }
            break;

        case OPCODE::MANUAL:

            ControlLoop(manualServoSetpoint_, 0);
            break;
        
        default:
            break;
    }
    ReadSensors();
    PublishServoCommand();
    PublishSensorsFb();
    PublishOperationState();
}