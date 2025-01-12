#ifndef PAYLOAD_CONTROL_HPP
#define PAYLOAD_CONTROL_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class PayloadControl
{
public:
    PayloadControl(ros::NodeHandle &nh);
    ~PayloadControl() = default;
    void Update();

private:
    void PublishServoCommand();
    void EncoderCallback(std_msgs::Float32 msg);
    void ControlLoop();
    void PayloadUp();
    void PayloadDown();
    void PayloadHold();
    
    ros::NodeHandle nh_;
    ros::Publisher servoCommandPub_;
    ros::Subscriber encoderSub_;
    float servoPosSetpoint_;
    float servoVelOutput_;
    float encoderVal_;

    //messages
    std_msgs::Float32 servoVelMsg_;

    // pid control
    float kp_;  
    //float kd_;

    //float prevError_;
    float curError_;

};

#endif