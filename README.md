# navio2_ros

This repository contains ROS packages for Navio2 autopilot shield for Raspberry Pi.

## Current features
A single node currently retrieve and publish all sensor data.

Publishers for:
- [x] IMU data (both MPU9250 and LSM9DS1) in `imu/mpu` and `imu/lsm` topics
- [x] Barometer data with temperatures in `barometer` and `temperature` topics
- [x] GPS data in `gps/data` and `gps/ublox_status` topics
- [ ] PPM radio data in `rc_input` topic
- [ ] ADC data in `adc` topic

Subscribers for:
- [ ] LED color controls from `led` topic
- [ ] PWM controls from `pwm/channel*` topic

Other interesting features:
- [ ] use CallbackGroups and Executors to multi-thread sensor processing
- [ ] split the sensor node in multiple nodes and compose them
- [ ] add an AHRS filter to IMU data

## Dev container
The dev container in `.devcontainer` enables to develop for Navio2 on any platform, both through Visual Studio Code and Clion.
The produced image is not intended to be used for deployment, but only for development.
If you need colcon in it, start from `ros:humble-ros-base`, otherwise from `ros:humble-ros-code` (by editing the Dockerfile accordingly).
You'll probably need to set `net=host` too in order to run some node and hear it outside Docker or in your local network.
