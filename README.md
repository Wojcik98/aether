# Aether - micromouse robot

Source code for the micromouse robot Aether.

Main module: STM32F466RE

ROS 2 Jazzy integration

Probably in future: micro-ROS publishing in the background for visualization.

# Development environment

## Prerequisites

### just

This repository uses [just](https://github.com/casey/just) to basically make aliases for common commands.
You can check [installation page](https://just.systems/man/en/installation.html) or try installing with:

```bash
mkdir -p ~/bin
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/bin
export PATH="$PATH:$HOME/bin"
```

### Docker

Install docker with instructions on their page.

To build all Docker images, run `just docker-build` (or `just bd`).
Base image is `ghcr.io/sloretz/ros:jazzy-desktop-full`.

TODO: download packages in Dockerfile so they don't need to download each time a container is started.

### VS Code

There are settings in this repository to make using it in VS Code easier.

## Building

### STM32

`build-stm` / `just b` - builds code for the STM32.

Flashing the device is currently only supported through the VS Code using extension.

### ROS 2

TODO
