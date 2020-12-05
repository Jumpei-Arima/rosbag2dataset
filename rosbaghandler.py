import collections
import numpy as np

import rospy
import rosbag


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

    def read_messages(self, topics=None, start_time=None, end_time=None, hz=None):
        if start_time is None:
            start_time = self.start_time
        if end_time is None:
            end_time = self.end_time
        start_time = rospy.Time.from_seconds(start_time)
        end_time = rospy.Time.from_seconds(end_time)
        data = {}
        topic_names = []
        for topic in topics:
            data[topic] = []
            topic_names.append("/"+topic)
        for topic, msg, time in self.bag.read_messages(topics=topic_names, start_time=start_time, end_time=end_time):
            if hz is not None:
                data[topic[1:]].append([time.to_nsec()/1e9, msg])
            else:
                data[topic[1:]].append(msg)
        if hz is not None:
            data = self.convert_data(data, hz)
        return data

    def get_topic_type(self, topic_name):
        topic_type = None
        for topic, topic_info in self.info.topics.items():
            if str(topic_name) == topic[1:]:
                topic_type = topic_info.msg_type
        return topic_type

    def convert_data(self, data, hz):
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