# navio2_ros

This repository contains ROS packages for Navio2 autopilot shield for Raspberry Pi.

## Dev container
The dev container in `.devcontainer` enables to develop for Navio2 on any platform, both through Visual Studio Code and Clion.
The produced image is not intended to be used for deployment, but only for development.
If you need colcon in it (not suggested), start from `ros:humble-ros-base` instead of `ros:humble-ros-code`, and you'll probably need to set `net=host` too in order to run some node.
