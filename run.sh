#!/usr/bin/bash

script_dir=$(cd "$(dirname $0)"; pwd)

# check if running as root, otherwise use sudo
if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
fi

# Check if user's current group includes spi
if ! groups | grep -q spi; then
  echo "WARN: User's current group does not include spi. Consider including it (usermod -aG spi $USER)."
  echo "In the meantime, I'll temporarily fix spi permissions for you."

  # Fix SPI permissions by overwriting device ownership
  $SUDO chown $(id -u):$(id -g) /dev/spi*
fi

docker run \
    --platform linux/arm/v7 \
    --rm -it \
    --privileged \
    --cap-add ALL \
    --net=host \
    --ulimit nofile=1024:524288 \
    -v /dev:/dev \
    -v "$script_dir":/home/docker/ws/src \
    navio2/armv7:latest
