#!/usr/bin/env python
import os
import sys
import argparse
import numpy as np
import time
from datetime import datetime

import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main(args, out_dir):
    rospy.init_node('rosbag2dataset', anonymous=True)
    print(__file__ + " start!!")
    print('bagfile: ' + args.bagfile_name)

    try:
        bag = rosbag.Bag(args.bagfile_name)
    except Exception as e:
        rospy.logfatal('failed to load bag file:%s', e)
        exit(1)

    topics = [args.image_topic, args.odom_topic]
    print(topics)
    bridge = CvBridge()
    velocities = []
    last_time = None
    for topic, msg, time in bag.read_messages(topics=topics):
        t = datetime.fromtimestamp(float(str(time.secs) + '.' + str(time.nsecs)))
        t = t.strftime('%Y%m%d%H%M%S%f')
        if topic==args.image_topic:
            if last_time is None:
                last_time = t
            try:
                img = bridge.imgmsg_to_cv2(msg,"bgr8")
                h,w,c = img.shape
                img = img[0:h,int((w-h)*0.5):w-int((w-h)*0.5),:]
                img = cv2.resize(img, (args.height, args.width))
                
                if len(velocities) > 0:
                    vel = np.mean(velocities, axis=0)
                    velocities = []  

                    if args.save:
                        img_file_name = out_dir + '/img/' + str(t) + '.jpg'
                        vel_file_name = out_dir + '/vel/' + str(last_time) + '.npy'
                        print(img_file_name)
                        print(vel_file_name)
                        cv2.imwrite(img_file_name, img)
                        np.save(vel_file_name, vel)
                        last_time = t
                    
                if args.show_image:
                    cv2.imshow("image", img)
                    cv2.waitKey(1)
            except CvBridgeError as e:
                print(e) 


        if topic==args.odom_topic:
            velocities.append([msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        if rospy.is_shutdown():
            exit()

    bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bagfile_name', type=str)
    parser.add_argument('--image_topic', type=str, default='/camera/color/image_raw')
    parser.add_argument('--odom_topic', type=str, default='/odom')
    parser.add_argument('--out_dir', type=str, default='dataset/')
    parser.add_argument('--width', type=int, default='472')
    parser.add_argument('--height', type=int, default='472')
    parser.add_argument('--show_image', action='store_true', default=False)
    parser.add_argument('--save', action='store_true', default=False)
    args = parser.parse_args()

    if args.bagfile_name is None:
        raise ValueError('set bagfile')
    file_name = args.bagfile_name[-23:-4]
    out_dir = os.path.join(args.out_dir,file_name)
    print(out_dir)
    os.makedirs(out_dir)
    os.makedirs(os.path.join(out_dir, "img"))
    os.makedirs(os.path.join(out_dir, "vel"))

    main(args, out_dir)
