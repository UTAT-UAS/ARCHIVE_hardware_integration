#include <Arduino.h>
#include "payload_control.hpp"

ros::NodeHandle_<ArduinoHardware, 5, 5, 150, 150>  nh;
PayloadControl *pld = nullptr;

// define some static vars
std_msgs::Float32 PayloadControl::setpointMsg_;
std_msgs::Float32 PayloadControl::hookMsg_;
PayloadControl* PayloadControl::instance_ = nullptr;

const unsigned int rate = 30;    

void setup() {
    pld = new PayloadControl(nh);
}

void loop() {
  // put your main code here, to run repeatedly:
    static unsigned long last_time = 0;
    unsigned long current_time = millis();

    if (current_time - last_time >= (1000/rate))
    {
        last_time = current_time;
        pld->Update();
    }
}