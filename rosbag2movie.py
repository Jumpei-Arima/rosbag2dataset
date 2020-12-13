#!/usr/bin/env python3
import os
import shutil
import argparse

import cv2

from rosbaghandler import RosbagHandler
from utils import *

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--bagfile', type=str, default='bagfiles/hoge.bag')
    parser.add_argument('--image-topic', type=str, default='/usb_camera/image_raw')
    parser.add_argument('--output-dir', type=str, default='bagfiles/movie')
    parser.add_argument('--frame-rate', type=float, default=30)
    args = parser.parse_args()

    rosbag_handler = RosbagHandler(args.bagfile)

    t0 = rosbag_handler.start_time
    dt = 100
    os.makedirs("/tmp/rosbag2dataset", exist_ok=True)

    idx = 0
    while t0<rosbag_handler.end_time:
        t1 = t0 + dt
        if t1>rosbag_handler.end_time:
            t1 = rosbag_handler.end_time

        data = rosbag_handler.read_messages(topics=[args.image_topic], 
                start_time=t0, end_time=t1)
        img_data = data[args.image_topic]
        topic_type = rosbag_handler.get_topic_type(args.image_topic)
        if topic_type == "sensor_msgs/CompressedImage":
            images = convert_CompressedImage(img_data)
        elif topic_type == "sensor_msgs/Image":
            images = convert_Image(img_data)
        for img in images:
            cv2.imwrite(os.path.join("/tmp/rosbag2dataset", str(idx)+".png"), img)
            idx+=1
        t0 = t1

    img = cv2.imread("/tmp/rosbag2dataset/0.png")
    h,w,c = img.shape
    file_name = os.path.splitext(os.path.basename(args.bagfile))[0]
    output_path = os.path.join(args.output_dir, file_name+"-"+args.image_topic.replace('/','_')+".mp4")
    print("output_path: ", output_path)
    out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc('m','p','4','v'), args.frame_rate, (w,h))
    for idx in range(len(os.listdir("/tmp/rosbag2dataset"))):
        img = cv2.imread(os.path.join("/tmp/rosbag2dataset", str(idx)+".png"))
        out.write(img)
    out.release()
    shutil.rmtree("/tmp/rosbag2dataset")

if __name__=="__main__":
    main()
