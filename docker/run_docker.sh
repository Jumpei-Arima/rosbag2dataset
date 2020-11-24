#!/bin/sh

docker run -it --rm \
    --privileged \
    -v ${PWD}/../:/root/ \
    -v ${RWRC_DATASET_DIR}:/root/dataset \
    --name rwrc20_rosbag2dataset \
    arima/rosbag2dataset \
    bash -c "python3 rosbag2dataset.py"
