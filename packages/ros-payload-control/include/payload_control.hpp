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
#include <geometry_msgs/PointStamped.h>
#include <Wire.h>
#include <NBHX711.h>
#include <position_controller.hpp>
#include <velocity_controller.hpp>

#define R 0.016
#define ROT2LIN 2 * PI * R * (1.0 / 600) // 2 * pi * r * gear ratio
// deadband: 1430 -> 1543 us
#define MOVEMENT_DOWN_THRESH 1430
#define MOVEMENT_UP_THRESH 1543

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
    ros::Publisher encoderVelPub_;
    ros::Publisher stateMsgPub_;
    ros::Publisher operationDonePub_;
    ros::Publisher forcePub_;
    ros::Publisher weightPub_;
    ros::Publisher servoVelocityPub_;

private:
    // ros //
    void PublishServoCommand();
    void PublishOperationState();
    void PublishSensorsFb();
    ros::NodeHandle nh_;
    std_msgs::Int32 stateMsg_;
    std_msgs::Bool operationDoneMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderLenFbMsg_;
    std_msgs::Float32 encoderVelMsg_;
    geometry_msgs::PointStamped forceMsg_;
    std_msgs::Float32 weightMsg_;

    // encoder //
    void EncoderSetup();
    static void IRAM_ATTR EncoderISR();
    void IRAM_ATTR ReadEncoder();

    //void ProcessEncoder();
    void ReadSensors();
    void ProcessEncoder();
    unsigned char encState_{0};
    int pinA_ = 25;
    int pinB_ = 33;

    volatile int encoderRawISR_{0}; // read from encoders
    volatile float encoderLenISR_{0};
    volatile float encoderLen_{0};
    volatile float lastEncoderLen_{0};
    volatile unsigned long lastEncoderChangeTime_{0};
    float encoderVel_{0};
    float alphaVelEnc_{0.06};

    float stoppedEncoderLen_{0.0};

    // force sensor //
    void ForceSensorSetup();
    void ForceRead();
    int force1AnalogPin_ = 15;
    int force2AnalogPin_ = 4;

    float filteredForce_{0};
    float force1_{0}; 
    float force2_{0};
    float rawForce1_{0};
    float rawForce2_{0};
    float alphaForce_{0.2};  // smoothing factor
    float forceThreshold_{10.0};
    
    // water level //

    double rawWeight_{0};
    double weight_{0};
    float lastWeight_{0};
    float lastLastWeight_{0};
    float weightThreshold{100};
    const int LOADCELL_DOUT_PIN = 18;
    const int LOADCELL_SCK_PIN = 5;
    int loadCellOffset_{0};
    bool loadCellIsTweaking_{false};
    int32_t loadCellTweakCount_{0}; 
    float weightAlpha_{0.8};
    NBHX711 loadcell_;

    void LoadCellSetup();
    void LoadCellRead();
    float Tare(int32_t num);

    // servo control //
    void HookControlLoop(float hookSetpoint);
    void PositionControlLoop(float lenSetpoint);
    void VelocityControlLoop(float rps);
    void contServoAttach();
    void contServoWrite(int us);
    Servo hookServo_;
    Servo contServo_; // continous servo added
    int contServoPin_ = 14;
    int hookServoPin_ = 23;
    float manualServoSetpoint_{0}; // position
    bool stopDrop_{false};

    // controller vars
    float dt_{0.033};  // 30 Hz

    // pi position controller
    float kpPos_{0.65};  // 25  
    float kiPos_{0}; //18
    float kdPos_{0};
    float intClampPos{0.02};
    float alphaPos_{0.1};
    float alphaPosTau_{0.1};
    float maxSpd_{0.15}; 
    float velOutput_{0}; // velocity

    // pi velocity controller
    float kpVel_{500};
    float kiVel_{50};
    float intClampVel_{400};
    float maxServoUsDelta_{500};
    float alphaVel_{0.1};   
    float alphaVelTau_{0.1};
    float servoOutput_{0}; 

    // controller objects
    PositionPID posPID_;
    VelocityPID velPID_;

    // state machine //
    enum class OPCODE {
        STOPPED,
        PICKUP,
        DISPENSE,
        RESET,
        MANUAL,
        RESET_PICKUP
    };
    enum class State {
        IDLE,
        UNSPOOL,
        WAIT,
        RESPOOL
    };
    void SwitchState(State state);
    void StopRetraction();
    void Stop();
    void Pickup();
    void ResetPickup();
    void Dispense();
    void Reset();
    void Manual();

    // states
    OPCODE operation_{OPCODE::STOPPED};
    State state_{State::IDLE};
    bool operationDone_{false};
    bool pickupReset_{false};
    float waitTimerStart_{0};
    float stateSwitchStart_{0};
    // ros parameters for easy tuning
    float pickupLen_{2.0};
    float pickupTime_{30};
    float dispenseLen_{0.03};
    float dispenseTime_{12};

    // other //
    static PayloadControl* instance_;

};

#endif // PAYLOAD_CONTROL_HPP
