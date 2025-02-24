#include "payload_control.hpp"

PayloadControl::PayloadControl(ros::NodeHandle &nh) 
 : nh_(nh),
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_),
 stateMsgPub_("/pld/state_fb", &stateMsg_),
 operationDonePub_("/pld/op_done", &operationDoneMsg_),
 forcePub_("/pld/force", &forceMsg_),
 waterlevelPub_("/pld/water", &waterlevelMsg_)
{
    // hardware setup
    nh_.getHardware()->setBaud(115200); 
    nh_.initNode();

    // servo setup
    contServoAttach();
    contServoWrite(0.0);
    hookServo_.attach(hookServoPin_);
    hookServo_.write(2000);

    // subs
    ros::Subscriber<std_msgs::Float32>setpointSub_("/pld/manual_command", PayloadControl::SetpointCb);
    ros::Subscriber<std_msgs::Int32>stateSub_("/pld/state_command", PayloadControl::RecieveStateCb);
    nh_.subscribe(setpointSub_);
    nh_.subscribe(stateSub_);

    // pubs
    //nh_.advertise(servoVelocityPub_);
    nh_.advertise(encoderLenPub_);
    nh_.advertise(stateMsgPub_);
    nh_.advertise(operationDonePub_);
    nh_.advertise(forcePub_);
    nh_.advertise(waterlevelPub_);

    instance_ = this;

    // setup sensors
    noInterrupts();
    attachInterrupt(digitalPinToInterrupt(pinA_), PayloadControl::EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB_), PayloadControl::EncoderISR, CHANGE);
    interrupts();
    forceSensorSetup();
    tofSensorSetup();
}

PayloadControl::~PayloadControl()
{
    
}

void PayloadControl::ControlLoop(float lenSetpoint, float hookSetpoint)
{
    // pi controller

    noInterrupts();
    curError_ = lenSetpoint - encoderLen_;  // length error
    interrupts();
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
    forceRead();
    // read tof sensor
    // tofRead();
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
                        contServoWrite(1.5);  // slowly retract
                        if (force_ >= 10 || (millis() - lastEncoderChangeTime_) > 500) { // if force is detected, stop
                            ControlLoop(0, 1);
                            operation_ = OPCODE::STOPPED;
                            operationDone_ = true;
                            encoderRaw_ = 0;
                        }
                    } 
                    break;
                // another state for spooling up really slowly
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
                    if (millis() - waitTimerStart_ >= 3 * 1000) {
                        SwitchState(State::RESPOOL);
                        contServoWrite(-1.5);  // slowly retract
                    }
                    break;
                case State::RESPOOL:
                    // slowly retract and wait for force sensor
                    noInterrupts();
                    contServoWrite(1.5);  // slowly retract
                    if (force_ >= 50) { // if force is detected, stop
                        ControlLoop(0, 1);
                        operation_ = OPCODE::STOPPED;
                        operationDone_ = true;
                        encoderRaw_ = 0;
                    }
                    interrupts();
                    break;
            }
            break;

        case OPCODE::RESET:
            state_ = State::RESPOOL;
            contServoWrite(3);  // slowly retract
            ControlLoop(0,0);
            // state transition condition
            noInterrupts();
            if (force_ >= 10 ) { // if force is detected, stop
                ControlLoop(0, 1);
                operation_ = OPCODE::STOPPED;
                operationDone_ = true;
                encoderRaw_ = 0;
            }
            interrupts();
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
    nh_.spinOnce();
}