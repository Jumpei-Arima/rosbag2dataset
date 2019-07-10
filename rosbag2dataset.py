#!/usr/bin/env python
import sys
import argparse
import numpy as np

import cv2

import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import torch

def main(args):
    # rospy.init_node('rosbag2dataset', anonymous=True)
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
    while not rospy.is_shutdown():
        try:
            for topic, msg, time in bag.read_messages(topics=topics):
                if topic==args.image_topic:
                    try:
                        img = bridge.imgmsg_to_cv2(msg,"rgb8")
                        # cv2.imshow("image", img)
                        # cv2.waitKey(1)
                    except CvBridgeError as e:
                        print(e) 

                if topic==args.odom_topic:
                    vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
                    print(vel)
                print(time)
        except Exception as e:
            rospy.logwarn("fail: %s", e)

        finally:
            bag.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--bagfile_name', type=str)
    parser.add_argument('--image_topic', type=str, default='/camera/color/image_raw')
    parser.add_argument('--odom_topic', type=str, default='/odom')
    args = parser.parse_args()

    if args.bagfile_name is None:
        raise ValueError('set bagfile')

    main(args)
