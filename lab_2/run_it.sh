#!/bin/bash

docker run -it --rm \
    --privileged \
    --net=host \
    --gpus all \
    --shm-size=2g \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e DISPLAY=$DISPLAY \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /run/dbus:/run/dbus \
    -v ./storage:/opt/metavision_sdk/storage \
    -u 0 \
    prophesee_sdk:evk4
