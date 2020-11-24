FROM ros:noetic-ros-core

RUN apt-get update && \
    apt-get install -y \
        python3-pip \
        libgl1-mesa-dev \
        ros-noetic-tf*

RUN pip3 install --upgrade pip

RUN pip3 install torch opencv-python tqdm

WORKDIR /root
