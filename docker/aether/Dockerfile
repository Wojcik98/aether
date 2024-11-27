ARG BASE_IMAGE=ghcr.io/sloretz/ros:jazzy-desktop-full
FROM ${BASE_IMAGE}

ARG USER_ID=ubuntu

RUN echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y \
    udev \
    libusb-1.0-0 \
    wget \
    curl \
    unzip \
    vim \
    less \
    openssh-client && \
    rm -rf /var/lib/apt/lists/*

# Install `just` tool to /usr/bin and generate bash completions
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/bin && \
    just --completions bash > /just-completions.bash

SHELL ["/bin/bash", "-c"]
COPY entrypoint.sh /aether_entrypoint.sh
RUN chmod +x /aether_entrypoint.sh

USER $USER_ID
WORKDIR /home/$USER_ID

RUN . <(curl https://aka.ms/vcpkg-init.sh -L)

ENTRYPOINT ["/aether_entrypoint.sh"]
CMD ["/bin/bash"]
