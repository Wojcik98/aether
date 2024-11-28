import 'docker/docker.just'

alias bd := docker-build
alias br := build-ros
alias b := build-stm
alias xs := xhost-stuff
alias l := launch

default:
    @just --list

build-stm:
    @echo "Building STM32 firmware..."
    @cbuild /workspaces/aether/stm/aether.csolution.yml --context-set

build-ros:
    #!/bin/bash
    echo "Building ROS 2 workspace..."
    cd ros_ws
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

launch:
    ros2 launch aether_bringup aether_sim.launch.py

[doc("Should be executed from host, not inside docker.")]
docker-build: docker-build-aether
    @echo "All docker images built!"

[doc("Should be executed from host, not inside docker.")]
xhost-stuff:
    @sudo xhost +local:docker
    @sudo xhost +
