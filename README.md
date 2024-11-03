# Hardware Integration

ROS package for AEAC hardware integration

## Development Philosophy

Everything here handles integration with hardware. Creates ROS nodes that publish to a topic for consumption by the flight stack, or subscribe to a topic for forwarding to hardware. (potentially services as well)

Ideally the `Controller` (and flight stack code in general) should care as little as possible wether they are running in simulation or on the real drone. By creating hardware interfaces here we can limit the points at which the software directly touches real hardware (a huge source of pain and suffering). I encourage the use of C++ since it offers efficiency, especially to embedded devices such as the Raspberry Pi since it has a much slower CPU for tasks. Ideally, hardware should slow down the stack as little as possible. 

## TODO

- Integrate new hardware
    For example, ongoing projects include:
    - 5G data link
    - Camera and sensor integration
- Automation 
    - ROS nodes need to be started manually from seperate launch files. Responsible for streamlining it for pushing to the embedded device.
    - Creating and researching services we can start from the nano to enable hardware interfaces (like a vpn service)

- Integration with Embedded Systems
    - Ensure codebase works outside of container (e.g. no absolute paths anywhere)
