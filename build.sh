#!/usr/bin/bash

docker build \
    -f Dockerfile.armv7 \
    -t navio2/armv7:latest \
    .
