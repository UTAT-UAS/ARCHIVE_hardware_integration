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
#include <VL6180X.h>
#include <NBHX711.h>

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
    float filteredVel_{0};
    float alphaVel_{0.06};

    float stoppedEncoderLen_{0.0};

    // force sensor //
    void ForceSensorSetup();
    void ForceRead();
    int force1AnalogPin_ = 13;
    int force2AnalogPin_ = 26;

    float filteredForce_{0};
    float force1_{0}; 
    float force2_{0};
    float rawForce1_{0};
    float rawForce2_{0};
    float alpha_{0.2};  // smoothing factor
    float forceThreshold_{20};
    
    // water level //

    float rawWeight_{0};
    float weight_{0};
    float lastWeight_{0};
    const int LOADCELL_DOUT_PIN = 18;
    const int LOADCELL_SCK_PIN = 5;
    int loadCellOffset_{0};
    NBHX711 loadcell_;
    bool loadCellIsTweaking_{false};
    int32_t loadCellTweakCount_{0}; 
    float weightAlpha_{0.2};

    void LoadCellSetup();
    void LoadCellRead();
    float Tare(int32_t num);

    // servo control //
    void PositionControlLoop(float lenSetpoint, float hookSetpoint);
    void VelocityControlLoop(float rps);
    void contServoAttach();
    void contServoWrite(int us);
    Servo hookServo_;
    Servo contServo_; // continous servo added
    int contServoPin_ = 14;
    int hookServoPin_ = 27;
    float manualServoSetpoint_{0}; // position

    // pi position controller
    float kp_{25};  
    float ki_{18};
    float dt_{0.033};  // 30 Hz
    float curError_;
    float lastError_;
    float integral_{0};
    float maxSpd_{5.759*2};  // 5.759 rad/s at max speed
    float servoOutput_{0}; 

    // pi velocity controller
    float curVelError_;
    float velIntegral_{0};
    float kpVel_{2};
    float kiVel_{1};
    float velOutput_{0}; // velocity

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
    float pickupLen_{2};
    float pickupTime_{30};
    float dispenseLen_{0.03};
    float dispenseTime_{7};

    // other //
    static PayloadControl* instance_;

};

#endif // PAYLOAD_CONTROL_HPP
