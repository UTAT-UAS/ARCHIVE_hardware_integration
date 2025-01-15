#ifndef PAYLOAD_CONTROL_HPP
#define PAYLOAD_CONTROL_HPP


#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

class PayloadControl
{
public:
    PayloadControl(ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150>&nh);
    ~PayloadControl();
    void Update();

private:
    static void SetpointCb(const std_msgs::Float32 &msg);
    void PublishServoCommand();
    void PublishEncoderFb();
    static void EncoderISR();
    void HandleEncoder();
    void ControlLoop();

    //messages
    static std_msgs::Float32 setpointMsg_;
    std_msgs::Float32 servoVelMsg_;
    std_msgs::Float32 encoderRawFbMsg_;
    std_msgs::Float32 encoderLenFbMsg_;

    //msg variables
    float servoSetpoint_{0}; // position
    float servoOutput_{90};  // velocity
    float encoderRaw_{0}; // read from encoders
    float encoderLen_{0};
    float conversion_ = 2 * PI * 0.5 * (1.0 / 20);

    // ROS
    ros::NodeHandle_<ArduinoHardware, 1, 5, 150, 150> nh_;
    ros::Publisher servoVelocityPub_;
    ros::Publisher encoderRawPub_;
    ros::Publisher encoderLenPub_;

    // servos
    Servo pldServo_;

    // encoder
    int pinA_ = 3;
    int pinB_ = 4;
    int lastA_{0};

    // pid control
    float kp_{1};  
    float kd_;
    float max_{180};

    float prevError_;
    float curError_;

    static PayloadControl* instance_;

};

#endif
