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
#include <Wire.h>
#include <VL6180X.h>

#define ROT2LIN 2 * PI * 0.02 * (1.0 / 20) // 2 * pi * r * gear ratio

class PayloadControl
{
public:
    PayloadControl();
    ~PayloadControl() = default;
    void SensorsSetup();
    void UpdatePayload();
    int getPinA() {return pinA_;}
    int getPinB() {return pinB_;}

    static void SetpointCb(const std_msgs::Float32 &msg);
    static void RecieveStateCb(const std_msgs::Int32 &msg);

    ros::Publisher encoderLenPub_;
    ros::Publisher stateMsgPub_;
    ros::Publisher operationDonePub_;
    ros::Publisher forcePub_;
    ros::Publisher waterlevelPub_;

private:
    // ros //
    void PublishServoCommand();
    void PublishOperationState();
    void PublishSensorsFb();
    ros::NodeHandle nh_;
    //ros::Publisher servoVelocityPub_;
    std_msgs::Int32 stateMsg_;
    std_msgs::Bool operationDoneMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderLenFbMsg_;
    std_msgs::Float32 forceMsg_;
    std_msgs::Float32 waterlevelMsg_;

    // sensors feedback //
    void EncoderSetup();
    static void IRAM_ATTR EncoderISR();
    void IRAM_ATTR ReadEncoder();

    //void ProcessEncoder();
    void ReadSensors();
    unsigned char encState_{0};
    int pinA_ = 2;
    int pinB_ = 4;
       

    volatile int encoderRaw_{0}; // read from encoders
    volatile float encoderLen_{0};
    volatile float lastEncoderLen_{0};
    volatile unsigned long lastEncoderChangeTime_{0};

    // force sensor
    void ForceSensorSetup();
    void ForceRead();
    int forceAnalogPin_ = 13;
    float force_{0}; 
    float rawForce_{0};
    float alpha_{0.1};  // smoothing factor
    
    // water level

    float waterlevel_{0};
    VL6180X tof_sensor_;
    void TofSensorSetup();
    void TofRead();
    

    // servo control //
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

#endif // PAYLOAD_CONTROL_HPP
