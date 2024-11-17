ARG BASE_IMAGE=aether/espidf
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt-get update -q && \
    apt-get install -yq sudo lsb-release gosu nano && \
    rm -rf /var/lib/apt/lists/*

ARG TZ_ARG=UTC
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get update && \
    apt-get install -y \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN . $IDF_PATH/export.sh && \
    pip3 --no-cache-dir install \
    catkin_pkg \
    lark-parser \
    colcon-common-extensions \
    importlib-resources \
    empy==3.3.4
