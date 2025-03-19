#include "payload_control.hpp"

PayloadControl::PayloadControl() 
 :
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_),
 encoderVelPub_("/pld/encoder_vel", &encoderVelMsg_),
 stateMsgPub_("/pld/state_fb", &stateMsg_),
 operationDonePub_("/pld/op_done", &operationDoneMsg_),
 forcePub_("/pld/force", &forceMsg_),
 weightPub_("/pld/water", &weightMsg_),
 servoVelocityPub_("/pld/servo_vel", &servoVelMsg_),
 loadcell_(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN),
 posPID_(dt_, maxSpd_, -maxSpd_, kpPos_, kiPos_, 0, intClampPos, alphaPos_, alphaPosTau_),
 velPID_(dt_, maxServoUsDelta_, -maxServoUsDelta_, kpVel_, kiVel_, 0, intClampVel_, alphaVel_, alphaVelTau_, -1)
{
    instance_ = this;
}

void PayloadControl::SensorsSetup()
{
    // servo setup
    contServoAttach();
    contServoWrite(1500);
    hookServo_.attach(hookServoPin_);
    hookServo_.writeMicroseconds(2000);

    EncoderSetup();
    ForceSensorSetup();
    LoadCellSetup();
}

void PayloadControl::ReadSensors()
{
    // process encoder
    ProcessEncoder();
    // read force sensor
    ForceRead();
    // read load cell
    LoadCellRead();
}

void PayloadControl::HookControlLoop(float hookSetpoint)
{
    hookServo_.writeMicroseconds(map(hookSetpoint, 0, 1, 1025, 1875));
}

void PayloadControl::PositionControlLoop(float lenSetpoint)
{
    velOutput_ = posPID_.compute(lenSetpoint, encoderLen_);
    servoOutput_ = velPID_.compute(velOutput_, encoderVel_);

    // write to servo
    contServoWrite(servoOutput_);
}

void PayloadControl::VelocityControlLoop(float velSp)
{
    servoOutput_ = velPID_.compute(velSp, encoderVel_);
    contServoWrite(servoOutput_);
}

void PayloadControl::SwitchState(State state)
{
    state_ = state;
}

void PayloadControl::Stop()
{
    PositionControlLoop(stoppedEncoderLen_);
    HookControlLoop(1);
    SwitchState(State::IDLE);
    operationDone_ = true;
}

void PayloadControl::Pickup()
{
    operationDone_ = false;
    switch (state_)
    {
        case State::IDLE:

            contServoWrite(1500);
            PositionControlLoop(encoderLen_);
            HookControlLoop(0);
            SwitchState(State::UNSPOOL);    
            stateSwitchStart_ = millis();
            break;

        case State::UNSPOOL:

            // if the encoder has reached the desired length with tolerances
            // wait for hook dynamics
            if (millis() - stateSwitchStart_ <= 1000) {
                lastWeight_ = weight_;
                waitTimerStart_ = millis();
                break;
            }
            PositionControlLoop(pickupLen_);
            HookControlLoop(0);
            if (millis() - waitTimerStart_ >= 1500) {
                lastWeight_ = weight_;
                waitTimerStart_ = millis();
            }
            if (abs(pickupLen_ - encoderLen_) < 0.1 || lastWeight_ - weight_ > 100) {
                if (stopDrop_ == false) {
                    stopDrop_ = true;
                    waitTimerStart_ = millis();
                }
            }
            if (stopDrop_) {
                if (millis() - waitTimerStart_ <= 500) { // wait for 500 ms for some slack
                    break;
                }
                stoppedEncoderLen_ = encoderLen_;
                SwitchState(State::WAIT);
            }
            break;

        case State::WAIT:
            PositionControlLoop(stoppedEncoderLen_);
            HookControlLoop(0);
            if (millis() - waitTimerStart_ >= pickupTime_ * 1000) {
                stopDrop_ = false;
                SwitchState(State::RESPOOL);
            }
            break;

        case State::RESPOOL:

            if (encoderLen_ > 0.15) {
                PositionControlLoop(0.10);  // go back to 0 height with encoder fb
                HookControlLoop(0);
                waitTimerStart_ = millis();
            }
            else {
                VelocityControlLoop(-0.02);  // slowly retract

                if (millis() - waitTimerStart_ <= 2500) {
                    break;
                }

                // if force or encoder has not changed for a certain time then stop
                if (force1_ >= forceThreshold_ && force2_ >= forceThreshold_ || (millis() - lastEncoderChangeTime_) > 700) { 
                    encoderRawISR_ = 0;
                    encoderLen_ = 0;
                    stoppedEncoderLen_ = 0;
                    operation_ = OPCODE::STOPPED;
                }
            } 
            break;
    }
    return;
}

void PayloadControl::Dispense()
{
    operationDone_ = false;
    switch (state_)
    {
        case State::IDLE:
            // enable hook servo
            PositionControlLoop(encoderLen_);
            HookControlLoop(1);
            SwitchState(State::UNSPOOL);    
            waitTimerStart_ = millis();
            break;
        case State::UNSPOOL:
            // wait for hook dynamics
            if (millis() - waitTimerStart_ <= 1000) {
                lastWeight_ = weight_;
                break;
            }
            PositionControlLoop(dispenseLen_);
            HookControlLoop(1);
            if (abs(dispenseLen_ - encoderLen_) < 0.005) {
                waitTimerStart_ = millis();
                SwitchState(State::WAIT);
                
            }
            break;
        case State::WAIT:
            PositionControlLoop(dispenseLen_);
            HookControlLoop(1);
            // logic here to check water levels
            // if (current water amount - last water amount > 600) {
            // if (loadCellIsTweaking_) {
            if (millis() - waitTimerStart_ >= dispenseTime_ * 1000) {
                SwitchState(State::RESPOOL);
                contServoWrite(MOVEMENT_UP_THRESH + 100);  // slowly retract
            }
            // }
            // if (lastWeight_ - weight_ > 400) {
            //     SwitchState(State::RESPOOL);
            //     contServoWrite(MOVEMENT_UP_THRESH + 100);  // slowly retract
            // }

            break;
        case State::RESPOOL:
            // slowly retract and wait for force sensor

            contServoWrite(MOVEMENT_UP_THRESH + 150);  // slowly retract at 1.5 rad/s upwards
            if (force1_ >= forceThreshold_ && force2_ >= forceThreshold_) { // if force is detected, stop
                encoderRawISR_ = 0;
                encoderLen_ = 0;
                stoppedEncoderLen_ = 0;
                operation_ = OPCODE::STOPPED;
            }
            break;
    }
    return;
}

void PayloadControl::Reset()
{
    operationDone_ = false;
    state_ = State::RESPOOL;
    VelocityControlLoop(-0.05);  // slowly retract
    hookServo_.writeMicroseconds(1100);
    
    // state transition condition
    if (force1_ >= forceThreshold_ && force2_ >= forceThreshold_) { // if force is detected, stop
        encoderRawISR_ = 0;
        encoderLen_ = 0;
        stoppedEncoderLen_ = 0;
        operation_ = OPCODE::STOPPED;
    }
    return;
}

void PayloadControl::Manual()
{
    operationDone_ = false;
    HookControlLoop(0);
    PositionControlLoop(manualServoSetpoint_);
    //VelocityControlLoop(manualServoSetpoint_);
    //contServoWrite(MOVEMENT_DOWN_THRESH);
    return;
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
            Stop();
            break;
        case OPCODE::PICKUP:
            Pickup();
            break;
        case OPCODE::DISPENSE:
            Dispense();
            break;
        case OPCODE::RESET:
            Reset();
            break;
        case OPCODE::MANUAL:
            Manual();
            break;
        default:
            break;
    }
    ReadSensors();
    PublishServoCommand();
    PublishSensorsFb();
    PublishOperationState();
}