#include "payload_control.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "payload_control");
    ros::NodeHandle nh;

    PayloadControl pld(nh);

    ros::Rate rate(30);
    while (ros::ok())
    {
        pld.Update();
        ros::spinOnce();
        rate.sleep();
    }
}
