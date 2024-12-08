import 'docker/docker.just'

alias bd := docker-build
alias br := build-ros
alias b := build-stm
alias xs := xhost-stuff
alias l := launch
alias dt := dev-terminal
alias ta := test-aether

default:
    @just --list

build-stm:
    @echo "Building STM32 firmware..."
    @cbuild /workspaces/aether/stm/aether.csolution.yml --context-set

build-ros:
    #!/bin/bash
    echo "Building ROS 2 workspace..."
    MONOREPO_DIR=$(pwd)
    cd ros_ws
    colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMONOREPO_DIR=$MONOREPO_DIR

test-aether:
    #!/bin/bash
    echo "Testing Aether..."
    cd ros_ws
    source install/setup.bash
    colcon test --ctest-args tests --packages-select aether_app
    colcon test-result --test-result-base build/aether_app --verbose

launch:
    ros2 launch aether_bringup aether_sim.launch.py

clean-ros:
    #!/bin/bash
    echo "Cleaning ROS 2 workspace..."
    cd ros_ws
    rm -rf build install log

[doc("Should be executed from host, not inside docker.")]
dev-terminal:
    @docker exec -it -w /workspaces/aether aether_dev env TERM=xterm-256color bash -l

[doc("Should be executed from host, not inside docker.")]
docker-build: docker-build-aether
    @echo "All docker images built!"

[doc("Should be executed from host, not inside docker.")]
xhost-stuff:
    @sudo xhost +local:docker
    @sudo xhost +
