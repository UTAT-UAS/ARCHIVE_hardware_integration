# Hardware Integration

ROS package for AEAC hardware integration

## Development Philosophy

Everything here handles integration with hardware. Creates ROS nodes that publish to a topic for consumption by the flight stack, or subscribe to a topic for forwarding to hardware. (potentially services as well)

Ideally the `Controller` (and flight stack code in general) should care as little as possible wether they are running in simulation or on the real drone. By creating hardware interfaces here we can limit the points at which the software directly touches real hardware (a huge source of pain and suffering).

## TODO

- Integrate new hardware
    - depth.ai camera on drone
    - rfd link
        - actual rfd transmission scheme
        - align with webviewer to allow monitoring of the actual drone
    - any new hardware needed for comp
- System to control the jetson
    - currently stuff on jetson must be started via entering it with ssh
        - then starting ros nodes manually
        - also ensure codebase works outside of container (e.g. no aboslute paths anywhere)
    - create system to control with purely rfd commands
    - if too difficult automate most of the process, (e.g. automatically create tmux session on boot and try to connect to mavros)
