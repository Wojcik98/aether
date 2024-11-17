# Aether - micromouse robot

Source code for the micromouse robot Aether.

Main module: ESP32-S3

ROS 2 Humble integration (micro-ROS publishing in the background for visualization)

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

### VS Code

There are settings in this repository to make using it in VS Code easier.

## Building

To build all Docker images, run `just docker-build`.
