# rosbag2dataset
Scripts for converting rosbag files to dataset for robot learning.

Rosbag data is converted as torch.tensor().

## Environment

- Ubuntu20.04, ROS noetic
- Docker image (ghcr.io/jumpei-arima/rosbag2dataset:latest)
  - If you want to launch docker contaienr by following command.
    - `./RUN_DOCKER_CONTAINER.sh`

## How to Use
### 1. Set the configuration for converting rosbag file.

As shown in [config.json](https://github.com/Jumpei-Arima/rosbag2dataset/blob/master/config.json),

set the information required for conversion, such as the topic name and the path to the bagfile directory.

### 2. Converting rosbag file to dataset.

`python3 rosbag2dataset.py --config config.json`

## Option
###  Converting rosbag to movie.

`python3 rosbag2movei.py --bagfile bagfiles/hoge.bag --image-topic /usb_cam/image_raw --output-dir dataset/movie`