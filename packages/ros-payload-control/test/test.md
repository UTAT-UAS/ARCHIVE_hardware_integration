### To connect ROS:

* rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=115200

### To send a command: 

* rostopic pub /pld/state_command std_msgs/Int32 "data: 4"
* rostopic pub /pld/manual_command std_msgs/Float32 "data: 1.0"


### debugging:

[Buffer size for ATmega168](https://robotics.stackexchange.com/questions/38380/using-rosserial-for-a-atmega168-arduino-based-motorcontroller)