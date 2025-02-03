#include "payload_control.hpp"

PayloadControl::PayloadControl(ros::NodeHandle_<ArduinoHardware, 5, 5, 150, 150> &nh) 
 : nh_(nh), servoVelocityPub_("/pld/servo_sent", &servoVelMsg_), 
 encoderRawPub_("/pld/encoder_raw", &encoderRawFbMsg_), 
 encoderLenPub_("/pld/encoder_len", &encoderLenFbMsg_),
 stateMsgPub_("/pld/state_fb", &stateMsg_),
 operationDonePub_("/pld/op_done", &operationDoneMsg_)
{
    // hardware setup
    nh_.getHardware()->setBaud(57600);
    nh_.initNode();

    // servo setup
    PayloadControl::contServoAttach();
    PayloadControl::contServoWrite(0.0);
    hookServo_.attach(hookServoPin_);
    
    attachInterrupt(digitalPinToInterrupt(pinA_), PayloadControl::EncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(pinB_), PayloadControl::EncoderISR, CHANGE);

    // subs
    ros::Subscriber<std_msgs::Float32>setpointSub_("/pld/manual_command", PayloadControl::SetpointCb);
    ros::Subscriber<std_msgs::Int32>stateSub_("/pld/state_command", PayloadControl::RecieveStateCb);
    nh_.subscribe(setpointSub_);
    //nh_.subscribe(hookSub_);

    // pubs
    nh_.advertise(servoVelocityPub_);
    nh_.advertise(encoderRawPub_);
    nh_.advertise(encoderLenPub_);

    instance_ = this;
}

PayloadControl::~PayloadControl()
{
    
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
    hookServo_.write(hookSetpoint * 90);
    PayloadControl::contServoWrite(servoOutput_);
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

            PayloadControl::contServoWrite(0);
            SwitchState(State::IDLE);
            operationDone_ = true;
            break;

        case OPCODE::PICKUP:
            operationDone_ = false;
            switch (state_)
            {
                case State::IDLE:
                    PayloadControl::contServoWrite(0);
                    SwitchState(State::UNSPOOL);    
                    break;
                case State::UNSPOOL:
                    // if the encoder has reached the desired length with tolerances
                    ControlLoop(pickupLen_, 0);
                    if (abs(pickupLen_ - encoderLen_) < 0.2) {
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
                    ControlLoop(-0.1, 0);  // go back to 0 height
                    if (abs(-0.1 - encoderLen_) < 0.5 && force_ != 0) { // if force is detected, stop
                        operation_ = OPCODE::STOPPED;
                        operationDone_ = true;
                        encoderRaw_ = 0;
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

                    break;
                case State::RESPOOL:
                    ControlLoop(-0.1, 1);  // go back to 0 height
                    if (abs(-0.1 - encoderLen_) < 0.5 && force_ != 0) { // if force is detected, stop
                        operation_ = OPCODE::STOPPED;
                        operationDone_ = true;
                        encoderRaw_ = 0;
                    }
                    break;
            }
            break;
        case OPCODE::RESET:
            ControlLoop(-0.1, 1);  // go back to 0 height
            if (abs(-0.1 - encoderLen_) < 0.5 && force_ != 0) { // if force is detected, stop
                operation_ = OPCODE::STOPPED;
                encoderRaw_ = 0;
                operationDone_ = true;
            }
            break;

        case OPCODE::MANUAL:
            ControlLoop(manualServoSetpoint_, 1);
            break;
        
        default:
            break;
    }
    
    PublishServoCommand();
    PublishEncoderFb();
    PublishOperationState();
    nh_.spinOnce();
}