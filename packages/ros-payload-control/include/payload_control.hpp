#ifndef PAYLOAD_CONTROL_HPP
#define PAYLOAD_CONTROL_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <Rotary.h>
#include <ServoTimer2.h>

class PayloadControl
{
public:
    PayloadControl(ros::NodeHandle_<ArduinoHardware, 5, 5, 80, 105> &nh);
    ~PayloadControl();
    void UpdatePayload();

private:
    // ros //
    void PublishServoCommand();
    void PublishOperationState();
    void PublishSensorsFb();
    ros::NodeHandle_<ArduinoHardware, 5, 5, 80, 105> nh_;
    ros::Publisher servoVelocityPub_;
    ros::Publisher encoderLenPub_;
    ros::Publisher stateMsgPub_;
    ros::Publisher operationDonePub_;
    ros::Publisher forcePub_;

    std_msgs::Int32 stateMsg_;
    std_msgs::Bool operationDoneMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderLenFbMsg_;
    std_msgs::Float32 forceMsg_;

    // sensors feedback //
    static void EncoderISR();
    void HandleEncoder();
    void ReadSensors();
    int pinA_ = 2;
    int pinB_ = 3;
    Rotary encoder_ = Rotary(pinA_, pinB_);
    float encoderRaw_{0}; // read from encoders
    float encoderLen_{0};
    float conversion_ = 2 * PI * 0.02 * (1.0 / 20);

    // force sensor

    float force_{0}; 
    int pinForce_ = A0;
    void forceSensorSetup();
    void forceRead();
    
    // water level
    

    // servo control //
    static void SetpointCb(const std_msgs::Float32 &msg);
    void ControlLoop(float lenSetpoint, float hookSetpoint);
    void contServoAttach();
    void contServoWrite(float rps);
    ServoTimer2 hookServo_;
    int contServoPin_ = 9;
    int hookServoPin_ = 11;
    float manualServoSetpoint_{0}; // position
    float servoOutput_{0};  // velocity

    // pi controller
    float kp_{25};  
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
    float pickupLen_{0.5};
    float pickupTime_{5.0};
    float dispenseLen_{0.03};
    float dispenseTime_{1.0};

    // other //
    static PayloadControl* instance_;

};

#endif
