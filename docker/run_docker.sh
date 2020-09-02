#!/bin/bash

BAGFILE_DIR=${HOME}/bagfiles

docker run -it --rm \
    -v ${PWD}/../:/root/scripts \
    -v ${BAGFILE_DIR}/:/root/bagfiles \
    --name rosbag2dataset \
    rosbag2dataset \
    bash
    # bash -c "cd /root/scripts; python rosbag2dataset.py --config ${CONFIG_FILE}"
