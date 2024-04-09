#!/usr/bin/bash

script_dir=$(cd $(dirname $0); pwd)

docker run \
    --platform linux/arm/v7 \
    --rm -it \
    --privileged \
    --net=host \
    --ulimit nofile=1024:524288 \
    -v /dev:/dev \
    -v $script_dir:/home/docker/ws/src \
    navio2/armv7:latest
