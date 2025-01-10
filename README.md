# Hardware Integration

ROS package for AEAC hardware integration

## Development Philosophy

Everything here handles integration with hardware. Creates ROS nodes that publish to a topic for consumption by the flight stack, or subscribe to a topic for forwarding to hardware. (potentially services as well)

Ideally, hardware should slow down the stack as little as possible. Therefore I encourage the use of C++ since it offers efficiency, especially when running multiple processes at once on the Jetson Orin. 

## Installation

1. Get all packages when inside the /dev-environment directory: `vcs import < ./repos/latest.repos`.
2. Update the submodules (other git repos from github) with `git submodule update --init --remote`.

## Example Projects

- gstreamer-ros-bridge: handles video streaming to ROS and the ground station.
- antenna-tracker: integrates the antenna tracker for direct LOS communication.
- ros-payload-retraction: handles the payload retraction mechanism with an onboard ESP32 & encoder. (TBD)
