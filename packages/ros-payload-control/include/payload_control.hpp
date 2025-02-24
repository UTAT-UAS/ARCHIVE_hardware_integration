#ifndef PAYLOAD_CONTROL_HPP
#define PAYLOAD_CONTROL_HPP

#include <Arduino.h>
#include <ESP32Servo.h>
#undef ESP32
#include <ros.h>
#define ESP32
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <Rotary.h>
#include <Wire.h>
#include <VL6180X.h>

#define ROT2LIN 2 * PI * 0.02 * (1.0 / 20) // 2 * pi * r * gear ratio

class PayloadControl
{
public:
    PayloadControl(ros::NodeHandle &nh);
    ~PayloadControl();
    void UpdatePayload();

private:
    // ros //
    void PublishServoCommand();
    void PublishOperationState();
    void PublishSensorsFb();
    ros::NodeHandle nh_;
    //ros::Publisher servoVelocityPub_;
    ros::Publisher encoderLenPub_;
    ros::Publisher stateMsgPub_;
    ros::Publisher operationDonePub_;
    ros::Publisher forcePub_;
    ros::Publisher waterlevelPub_;

    std_msgs::Int32 stateMsg_;
    std_msgs::Bool operationDoneMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderLenFbMsg_;
    std_msgs::Float32 forceMsg_;
    std_msgs::Float32 waterlevelMsg_;

    // sensors feedback //
    static void EncoderISR();
    void HandleEncoder();
    void ReadSensors();
    int pinA_ = 26;
    int pinB_ = 25;
    Rotary encoder_ = Rotary(pinA_, pinB_);
    float encoderRaw_{0}; // read from encoders
    float encoderLen_{0};
    float lastEncoderLen_{0};
    unsigned long lastEncoderChangeTime_{0};

    // force sensor
    void forceSensorSetup();
    void forceRead();
    int forceAnalogPin_ = 12;
    float force_{0}; 
    float rawForce_{0};
    float alpha_{0.1};  // smoothing factor
    
    // water level

    float waterlevel_{0};
    VL6180X tof_sensor_;
    void tofSensorSetup();
    void tofRead();
    

    // servo control //
    static void SetpointCb(const std_msgs::Float32 &msg);
    void ControlLoop(float lenSetpoint, float hookSetpoint);
    void contServoAttach();
    void contServoWrite(float rps);
    Servo hookServo_;
    Servo contServo_; // continous servo added
    int contServoPin_ = 14;
    int hookServoPin_ = 27;
    float manualServoSetpoint_{0}; // position
    float servoOutput_{0};  // velocity

    // pi controller
    float kp_{50};  
    //float kd_{1};
    float ki_{2};
    float dt_{0.033};  // 30 Hz
    float curError_;
    float lastError_;
    float integral_{0};
    float maxSpd_{5.759};  // 5.759 rad/s at max speed

    // state machine //
    enum class OPCODE {
        STOPPED,
        PICKUP,
        DISPENSE,
        RESET,
        MANUAL
    };
    enum class State {
        IDLE,
        UNSPOOL,
        WAIT,
        RESPOOL
    };
    static void RecieveStateCb(const std_msgs::Int32 &msg);
    void SwitchState(State state);
    OPCODE operation_{OPCODE::STOPPED};
    State state_{State::IDLE};
    bool operationDone_{false};
    float waitTimerStart_{0};
    // ros parameters for easy tuning
    float pickupLen_{0.8};
    float pickupTime_{5.0};
    float dispenseLen_{0.03};
    float dispenseTime_{1.0};

    // other //
    static PayloadControl* instance_;

};

#endif
