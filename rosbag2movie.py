#!/usr/bin/env python3
import os
import argparse

import numpy as np
import cv2
import rosbag

from rosbaghandler import RosbagHandler
from utils import *

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bagfile', type=str, default='bagfiles/hoge.bag')
    parser.add_argument('--image-topic', type=str, default='/usb_camera/image_raw')
    parser.add_argument('--output-dir', type=str, default='bagfiles/movie')
    args = parser.parse_args()

    rosbag_handler = RosbagHandler(args.bagfile)
    data = rosbag_handler.read_messages(topics=[args.image_topic])
    img_data = data[args.image_topic]
    topic_type = rosbag_handler.get_topic_type(args.image_topic)
    images = [];
    if topic_type == "sensor_msgs/CompressedImage":
        images = convert_CompressedImage(img_data)
    elif topic_type == "sensor_msgs/Image":
        images = convert_Image(img_data)

    h,w,c = images[0].shape
    file_name = os.path.splitext(os.path.basename(args.bagfile))[0]
    output_path = os.path.join(args.output_dir, file_name+".mp4")
    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc('m','p','4','v'), 30, (w,h))
    for i in range(len(images)):
        out.write(images[i])
    out.release()

if __name__=="__main__":
    main()
