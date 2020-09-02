FROM ros:noetic-ros-core

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        ros-noetic-tf* \
        libgl1-mesa-dev \
        libgl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

RUN pip3 install future torch opencv-python

WORKDIR /root
