#!/usr/bin/bash

script_dir=$(cd "$(dirname $0)"; pwd)

# check if running as root, otherwise use sudo
if [ "$EUID" -ne 0 ]; then
    SUDO="sudo"
fi

fix_permissions() {
  local interface=$1

  # Check if user's current group includes the specified interface
  if ! groups | grep -q $interface; then
    echo "WARN: User's current group does not include $interface. Consider including it (usermod -aG $interface $USER)."
    echo "In the meantime, I'll temporarily fix $interface permissions for you."

    # Fix interface permissions by overwriting device ownership
    $SUDO chown $(id -u):$(id -g) /dev/$interface*
  fi
}

fix_permissions spi
fix_permissions i2c

SPI_GID=$(getent group spi | cut -d: -f3)
I2C_GID=$(getent group i2c | cut -d: -f3)

docker run \
    --platform linux/arm/v7 \
    --rm -it \
    --privileged \
    --cap-add ALL \
    --group-add $SPI_GID \
    --group-add $I2C_GID \
    --net=host \
    --ulimit nofile=1024:524288 \
    -v /dev:/dev \
    -v /etc/group:/etc/group:ro \
    -v "$script_dir":/home/docker/ws/src \
    navio2/armv7:latest
    # -v /etc/passwd:/etc/passwd:ro \
