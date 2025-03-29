## ROS Payload Control

`rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600`
`rostopic pub /pld/servo_command std_msgs/Float32 "data: 1.0"`