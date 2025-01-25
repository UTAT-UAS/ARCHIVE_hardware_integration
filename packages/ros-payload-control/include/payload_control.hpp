#ifndef PAYLOAD_CONTROL_HPP
#define PAYLOAD_CONTROL_HPP

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Rotary.h>
#include <Servo.h>


class PayloadControl
{
public:
    PayloadControl(ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150>&nh);
    ~PayloadControl();
    void Update();

private:
    static void SetpointCb(const std_msgs::Float32 &msg);
    static void HookCb(const std_msgs::Float32 &msg);
    void PublishServoCommand();
    void PublishEncoderFb();
    static void EncoderISR();
    void HandleEncoder();
    void ControlLoop();

    //messages
    static std_msgs::Float32 setpointMsg_;
    static std_msgs::Float32 hookMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderRawFbMsg_;
    std_msgs::Float32 encoderLenFbMsg_;

    //msg variables
    float servoSetpoint_{0}; // position
    float servoOutput_{0};  // velocity
    float encoderRaw_{0}; // read from encoders
    float encoderLen_{0};
    float conversion_ = 2 * PI * 0.02 * (1.0 / 20);

    // ROS
    ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150> nh_;
    ros::Publisher servoVelocityPub_;
    ros::Publisher encoderRawPub_;
    ros::Publisher encoderLenPub_;

    // servos
    int contServoPin_ = 9;
    int hookServoPin_ = 10;
    void contServoAttach();
    void contServoWrite(float rps);
    Servo hookServo_;

    // encoder
    int pinA_ = 2;
    int pinB_ = 3;
    Rotary encoder_ = Rotary(pinA_, pinB_);

    
    // pd control
    float kp_{25};  
    //float kd_{1};
    float ki_{2};
    float dt_{0.033};  // 30 Hz
    float curError_;
    float lastError_;
    float integral_{0};
    float maxSpd_{5.759};  // 5.759 rad/s at max speed

    static PayloadControl* instance_;

};

#endif
