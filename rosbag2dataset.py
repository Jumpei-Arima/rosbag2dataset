#!/usr/bin/env python
import os
import argparse
import collections 
import json
import numpy as np

import rospy
import rosbag
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
import tf
import torch
import cv2

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

class RosbagHandler:
    def __init__(self, bagfile):
        print('bagfile: ' + bagfile)
        try:
            self.bag = rosbag.Bag(bagfile)
        except Exception as e:
            rospy.logfatal('failed to load bag file:%s', e)
            exit(1)
        TopicTuple = collections.namedtuple("TopicTuple", ["msg_type", "message_count", "connections", "frequency"]) 
        TypesAndTopicsTuple =  collections.namedtuple("TypesAndTopicsTuple", ["msg_types", "topics"])
        self.info = self.bag.get_type_and_topic_info()
        for topic, topic_info in self.info.topics.items():
            print("======================================================")
            print("topic_name:      " + topic)
            print("topic_msg_type:  " + topic_info.msg_type)
            print("topic_msg_count: " + str(topic_info.message_count))
            print("frequency:       " + str(topic_info.frequency))
        self.start_time = self.bag.get_start_time()
        self.end_time = self.bag.get_end_time()
        print("start time: " + str(self.start_time))
        print("end time:   " + str(self.end_time))

    def read_messages(self, topics=None, start_time=None, end_time=None):
        start_time = rospy.Time.from_seconds(start_time)
        end_time = rospy.Time.from_seconds(end_time)
        data = {}
        topic_names = []
        for topic in topics:
            data[topic] = []
            topic_names.append("/"+topic)
        for topic, msg, time in self.bag.read_messages(topics=topic_names, start_time=start_time, end_time=end_time):
            data[topic[1:]].append([time.to_nsec()/1e9, msg])
        return data

    def get_topic_type(self, topic_name):
        topic_type = None
        for topic, topic_info in self.info.topics.items():
            if str(topic_name) == topic[1:]:
                topic_type = topic_info.msg_type
        return topic_type

def convert_data(data, hz):
    data_ = {}
    start_time = 0
    end_time = np.inf 
    idx = {}
    for topic in data.keys():
        start_time = max(start_time, data[topic][0][0])
        end_time = min(end_time, data[topic][-1][0])
        data_[topic] = []
        idx[topic] = 1
    t = start_time
    while(t<end_time):
        for topic in data.keys():
            cnt = 0
            while(data[topic][idx[topic]][0]<t):
                cnt+=1
                idx[topic]+=1
            if (data[topic][idx[topic]][0]-t<t-data[topic][idx[topic]-1][0]):
                data_[topic].append(data[topic][idx[topic]][1])
            else:
                data_[topic].append(data[topic][idx[topic]-1][1])
        t+=1./hz
    return data_

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default="configs/config.json")
    parser.add_argument('--bagfile', type=str, default="bagfile/example.bag")
    parser.add_argument('--out_dir', type=str, default="dataset")
    args = parser.parse_args()

    # config load
    if not os.path.exists(args.config):
        raise ValueError("cannot find config file")
    with open(args.config, "r") as f:
        config = json.load(f)

    # bagfile
    if not os.path.exists(args.bagfile):
        raise ValueError('set bagfile')
    file_name = os.path.splitext(os.path.basename(args.bagfile))[0]
    print('file_name: ', file_name)
    out_dir = os.path.join(args.out_dir,file_name)
    if(not os.path.exists(out_dir)):
        os.makedirs(out_dir)
    else:
        raise ValueError('set out_dir')
    print("out_dir: ", out_dir)

    rosbag_handler = RosbagHandler(args.bagfile)

    Traj = collections.namedtuple('Traj', ['observations', 'actions', 'infos'])
    time_per_iteration = config["steps_per_trajectory"] / config["hz"]
    t0 = rosbag_handler.start_time
    t1 = t0 + time_per_iteration
    num_traj = 0
    while(t1<rosbag_handler.end_time):
        print("traj: %d " % num_traj)
        print("time: %s to %s" % (t0, t1))
        topics = config["observation_topic"] + config["action_topic"] + config["info_topics"]
        print("topics: " , topics)
        data = rosbag_handler.read_messages(topics=topics, start_time=t0, end_time=t1)
        sample_data = convert_data(data, config["hz"])
        ob = []
        ac = []
        po = []
        for topic in sample_data.keys():
            topic_type = rosbag_handler.get_topic_type(topic)
            if topic_type == "sensor_msgs/LaserScan":
                for data in sample_data[topic]:
                    min_idx = int((-np.pi*0.5 - data.angle_min) / data.angle_increment)
                    max_idx = len(data.ranges) - int((data.angle_max - np.pi*0.5) / data.angle_increment)
                    if max_idx-min_idx < config["lidar_size"]:
                        continue
                    kernel_size = int((max_idx - min_idx) / config["lidar_size"])
                    ranges = []
                    for i,r in enumerate(data.ranges):
                        if not data.range_min < r < data.range_max:
                            ranges.append(data.range_max)
                        else:
                            ranges.append(r)
                    ranges = np.array(ranges[min_idx:int(min_idx+kernel_size*config["lidar_size"])])
                    ranges = ranges.reshape([-1, kernel_size])
                    ranges = np.amin(ranges, axis=1).tolist()
                    ob.append(ranges)
            elif topic_type == "sensor_msgs/CompressedImage":
                for data in sample_data[topic]:
                    img = cv2.imdecode(np.fromstring(data.data, np.uint8), cv2.IMREAD_COLOR)
                    h,w,c = img.shape
                    img = img[0:h,int((w-h)*0.5):w-int((w-h)*0.5),:]
                    img = cv2.resize(img, (config["height"], config["width"]))
                    ob.append(img)
            elif topic_type == "nav_msgs/Odometry":
                init_yaw = quaternion_to_euler(sample_data[topic][0].pose.pose.orientation).z
                p0 = np.array([sample_data[topic][0].pose.pose.position.x, sample_data[topic][0].pose.pose.position.y, init_yaw])
                for data in sample_data[topic]:
                    vel = np.array([data.twist.twist.linear.x, data.twist.twist.angular.z])
                    ac.append(vel)
                    yaw = quaternion_to_euler(data.pose.pose.orientation).z
                    p_abs = np.array([data.pose.pose.position.x, data.pose.pose.position.y, yaw])
                    p = p_abs-p0
                    p[2] = np.arctan2(np.sin(p[2]), np.cos(p[2]))
                    po.append(p)
        file_name = ("%d.pt" % (num_traj))
        path = os.path.join(out_dir, file_name)
        obs = torch.tensor(ob, dtype=torch.float32)
        acs = torch.tensor(ac, dtype=torch.float32)
        pos = torch.tensor(po, dtype=torch.float32)
        traj_data = Traj(obs, acs, pos)
        with open(path, "wb") as f:
            torch.save(traj_data, f)
        t0=t1
        t1=t0+time_per_iteration
        num_traj+=1
