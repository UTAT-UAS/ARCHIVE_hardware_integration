#include "payload_control.hpp"

ros::NodeHandle nh;
PayloadControl *pld = nullptr;

// Define static instance pointer
PayloadControl* PayloadControl::instance_ = nullptr;

const unsigned int rate = 30;  // Update rate in Hz

void setup() {
    pld = new PayloadControl(nh);
}


void loop() {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();

    if (current_time - last_time >= (1000 / rate)) {
        last_time = current_time;
        pld->UpdatePayload();  
        nh.spinOnce(); 
    }
}
