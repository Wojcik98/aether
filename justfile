import 'docker/docker.just'

alias bd := docker-build
alias f := flash-esp
alias b := build-esp

default:
    @just --list

build-esp: esp-generate-sdkconfig
    @echo "Building ESP32 firmware..."
    @cd esp && idf.py build

flash-esp:
    @echo "Flashing ESP32 firmware..."
    @cd esp && idf.py -p /dev/ttyACM0 flash

config-esp:
    @echo "Configuring ESP32 firmware..."
    @cd esp && idf.py menuconfig

build-ros:
    #!/bin/bash
    echo "Building ROS 2 workspace..."
    cd ros_ws
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Should be executed from host
micro-agent:
    @echo "Running micro-ROS agent..."
    docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6 -d

# Should be executed from host
docker-build: docker-build-aether
    @echo "All docker images built!"

[private]
esp-generate-sdkconfig:
    @cd esp && cat sdkconfig.defaults.in sdkconfig.secrets > sdkconfig.defaults
