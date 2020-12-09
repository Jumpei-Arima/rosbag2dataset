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
    file_name = os.path.splitext(os.path.basename(bagfile))[0]+"_traj"+str(config["traj_steps"])
    out_dir = os.path.join(config["output_dir"], file_name)
    print("out_dir: ", out_dir)
    os.makedirs(out_dir, exist_ok=True)
    for data_name in config["dataset"]:
        os.makedirs(os.path.join(out_dir, data_name), exist_ok=True)
    rosbag_handler = RosbagHandler(bagfile)

    t0 = rosbag_handler.start_time
    t1 = rosbag_handler.end_time
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
            print("==== convert odometry ====")
            dataset['acs'], dataset['pos'] = \
                convert_Odometry(sample_data[topic], config['action_noise'],
                                    config['lower_bound'], config["upper_bound"])
        elif topic_type == "geometry_msgs/Twist":
            print("==== convert twist ====")
            dataset['acs'] = convert_Twist(sample_data[topic], config['action_noise'], config['lower_bound'], config["upper_bound"])
        elif topic_type == "sensor_msgs/LaserScan":
            print("==== convert laser scan ====")
            dataset["lidar"] = convert_LaserScan(sample_data[topic])
        elif topic_type == "sensor_msgs/Imu":
            print("==== convert imu ====")
            dataset["imu"] = convert_Imu(sample_data[topic])

    print("==== save data as torch tensor ====")
    if "goal" in config["dataset"]:
        num_steps = len(dataset["acs"]) - config["goal_steps"]
    else:
        num_steps = len(dataset["acs"])
    num_traj = int(num_steps/config["traj_steps"])
    for idx in tqdm(range(num_traj)):
        file_name = ("%d.pt" % (idx))
        t0 = idx*config["traj_steps"]
        t1 = t0+config["traj_steps"]
        for data_name in config["dataset"]:
            path = os.path.join(out_dir, data_name, file_name)
            if data_name == "pos":
                traj_pos = dataset["pos"][t0:t1]
                poses = []
                init_pose = traj_pos[0].copy()
                for idx, pose in enumerate(traj_pos):
                    trans_pose = transform_pose(pose, init_pose)
                    poses.append(trans_pose)
                data = torch.tensor(poses, dtype=torch.float32)
            elif data_name == "goal":
                traj_pos = dataset["pos"][t0:t1+config["goal_steps"]]
                goals = []
                for idx, pose in enumerate(traj_pos):
                    if idx+config["goal_steps"]<len(traj_pos):
                        goal = transform_pose(traj_pos[idx+config["goal_steps"]], pose)
                        goals.append(goal)
                data = torch.tensor(goals, dtype=torch.float32)
            else:
                traj_data = dataset[data_name][t0:t1]
                data = torch.tensor(traj_data, dtype=torch.float32)
            with open(path, "wb") as f:
                torch.save(data, f)
    
    with open(os.path.join(out_dir, 'info.txt'), 'w') as f:
        info = config
        info['num_steps'] = num_steps
        info['num_traj'] = num_traj
        json.dump(info, f)
