#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
from audio_common_msgs.msg import AudioData
import numpy as np


class Data2Buffer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '{}' name ...".format(name))
        self.pub = rospy.Publisher("~audio_buffer", AudioBuffer, queue_size=10)
        rospy.Subscriber("/audio/audio", AudioData, self.callback)
        rospy.loginfo("started '{}'.".format(name))

    def callback(self, msg):
        a = AudioBuffer()
        b = bytearray(len(msg.data))
        b[:] = msg.data
        data = np.frombuffer(b, dtype=np.uint16)
        print type(data)
        a.header.stamp = rospy.Time.now()
        data = np.array(b, dtype=np.uint16)#.reshape(-1,2)[:,0]
        a.data = data.tolist()
        self.pub.publish(a)


if __name__ == "__main__":
    rospy.init_node("ad_to_ab_node")
    d = Data2Buffer(rospy.get_name())
    rospy.spin()

