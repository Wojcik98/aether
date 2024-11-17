ARG BASE_IMAGE=aether/microros
FROM ${BASE_IMAGE}

ARG USER_ID=ubuntu

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install `just` tool to /usr/bin
RUN curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to /usr/bin && \
    just --completions bash > /just-completions.bash

RUN apt-get update && apt-get install -y \
    udev && \
    rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /aether_entrypoint.sh
RUN ["chmod", "+x", "/aether_entrypoint.sh"]

USER $USER_ID
WORKDIR /home/$USER_ID

ENTRYPOINT ["/aether_entrypoint.sh"]
CMD ["/bin/bash"]
