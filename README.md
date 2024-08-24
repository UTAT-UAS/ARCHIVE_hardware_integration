# Hardware Integration

ROS package for AEAC hardware integration

## Development Philosophy

Everything here handles integration with hardware/sim. The rest of the libraries should never have to touch hardware raw.

## TODO

- Integrate new hardware
    - depth.ai camera on drone
    - rfd link
        - actual rfd transmission scheme
- Improve sim debugging
    - movable markers
        - visualize position tracking etc.
    - remotely move drone/objects
    - add comp objects (hanging bucket, ir thingies)