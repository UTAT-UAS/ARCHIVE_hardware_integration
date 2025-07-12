#include "payload_control.hpp"

PayloadControl* PayloadControl::instance_ = nullptr;
ros::NodeHandle nh;
PayloadControl pld;

ros::Subscriber<std_msgs::Float32>setpointSub_("/pld/manual_command", PayloadControl::SetpointCb);
ros::Subscriber<std_msgs::Int32>stateSub_("/pld/state_command", PayloadControl::RecieveStateCb);

// Define static instance pointer

const unsigned int rate = 30;  // Update rate in Hz

void setup() {
    // hardware setup
    nh.getHardware()->setBaud(115200); 
    nh.initNode();

    // subs
    nh.subscribe(setpointSub_);
    nh.subscribe(stateSub_);

    // pubs
    //nh_.advertise(servoVelocityPub_);
    nh.advertise(pld.encoderLenPub_);
    nh.advertise(pld.encoderVelPub_);
    nh.advertise(pld.stateMsgPub_);
    nh.advertise(pld.operationDonePub_);
    nh.advertise(pld.forcePub_);
    nh.advertise(pld.weightPub_);
    nh.advertise(pld.servoVelocityPub_);

    pld.SensorsSetup();
}


void loop() {
    static unsigned long last_time = 0;
    unsigned long current_time = millis();

    if (current_time - last_time >= (1000 / rate)) {
        last_time = current_time;
        pld.UpdatePayload();  
        nh.spinOnce(); 
    }
}
