#!/usr/bin/env python3
import os
import argparse
import json
from tqdm import tqdm

import torch

from rosbaghandler import RosbagHandler
from utils import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="config.json")
    args = parser.parse_args()

    if os.path.exists(args.config):
        with open(args.config, "r") as f:
            config = json.load(f)
    else:
        raise ValueError("cannot find config file")

    bagfile = os.path.join(config["bagfile_dir"], config["bagfile_name"]) 
    if not os.path.exists(bagfile):
        raise ValueError('set bagfile')
    file_name = os.path.splitext(os.path.basename(bagfile))[0]
    out_dir = os.path.join(config["output_dir"], file_name)
    print("out_dir: ", out_dir)
    os.makedirs(out_dir, exist_ok=True)
    for data_name in config["dataset"]:
        os.makedirs(os.path.join(out_dir, data_name), exist_ok=True)
    rosbag_handler = RosbagHandler(bagfile)

    t0 = rosbag_handler.start_time
    t1 = rosbag_handler.end_time
    num_step = 0
    sample_data = rosbag_handler.read_messages(topics=config["topics"], start_time=t0, end_time=t1, hz=config["hz"])
    dataset = {}
    for topic in sample_data.keys():
        topic_type = rosbag_handler.get_topic_type(topic)
        if topic_type == "sensor_msgs/CompressedImage":
            print("==== convert compressed image ====")
            dataset["obs"] = convert_CompressedImage(sample_data[topic], config["height"], config["width"])
        elif topic_type == "":
            print("==== convert image ====")
            dataset["obs"] = convert_Image(sample_data[topic], config["height"], config["width"])
        elif topic_type == "nav_msgs/Odometry":
            print("==== convert odom ====")
            dataset['acs'], dataset['pos'], dataset['goal'] = \
                convert_Odometry(sample_data[topic], config['action_noise'],
                                    config['lower_bound'], config["upper_bound"], config['goal_steps'])
        elif topic_type == "geometry_msgs/Twist":
            print("==== convert twist ====")
            dataset['acs'] = convert_Twist(sample_data[topic], config['action_noise'], config['lower_bound'], config["upper_bound"])
        elif topic_type == "sensor_msgs/LaserScan":
            print("==== convert lidar ====")
            dataset["lidar"] = convert_LaserScan(sample_data[topic])
        elif topic_type == "sensor_msgs/Imu":
            print("==== convert imu ====")
            dataset["imu"] = convert_Imu(sample_data[topic])

    print("==== save data as torch tensor ====")
    if "goal" in config["dataset"]:
        data_size = len(dataset["goal"])
    else:
        data_size = len(dataset["obs"])
    for idx in tqdm(range(data_size)):
        file_name = ("%d.pt" % (num_step))
        for data_name in config["dataset"]:
            path = os.path.join(out_dir, data_name, file_name)
            data = torch.tensor(dataset[data_name][idx], dtype=torch.float32)
            with open(path, "wb") as f:
                torch.save(data, f)
        num_step+=1
    
    with open(os.path.join(out_dir, 'info.txt'), 'w') as f:
        info = config
        info['num_step'] = num_step
        json.dump(config, f)