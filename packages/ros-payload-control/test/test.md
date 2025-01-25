### To connect ROS:

* rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600

### To send a command: 

* rostopic pub /pld/servo_command std_msgs/Float32 "data: 1.0"