#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from naoqi_bridge_msgs.msg import AudioBuffer
# from Queue import Queue, Empty
from future.moves import queue


SPEED_OF_SOUND = 343.3
CHUNK = 1024
mics = {
    AudioBuffer.CHANNEL_REAR_LEFT: [-0.0375, -0.0375, 0.0],
    AudioBuffer.CHANNEL_REAR_RIGHT: [0.0375, -0.0375, 0.0],
    AudioBuffer.CHANNEL_FRONT_LEFT: [-0.0375, 0.0375, 0.0],
    AudioBuffer.CHANNEL_FRONT_RIGHT: [0.0375, 0.0375, 0.0]
}


class DSBQueue(list):
    def put(self, item):
        try:
            self.extend(item)
        except TypeError:
            self.append(item)

    def get(self, size=1, block=True):
        while not rospy.is_shutdown():
            if self.__len__() < size:
                if not block:
                    raise queue.Empty()
                else:
                    rospy.sleep(.001)
                    continue
            res = self[:size]
            del self[:size]
            return res


class DsBeamformer(object):
    def __init__(self, name):
        rospy.loginfo("Starting {} ...".format(name))
        self._buffer = {
            AudioBuffer.CHANNEL_REAR_LEFT: DSBQueue(),
            AudioBuffer.CHANNEL_REAR_RIGHT: DSBQueue(),
            AudioBuffer.CHANNEL_FRONT_LEFT: DSBQueue(),
            AudioBuffer.CHANNEL_FRONT_RIGHT: DSBQueue()
        }
        self.channel_map = None
        self.frequency = None
        self.chunk_size = 0

        self.source = self.to_unit_vector(self.to_radians(90), self.to_radians(0))
        rospy.Subscriber("/naoqi_driver_node/audio", AudioBuffer, self.callback, queue_size=1)
        self.pub = rospy.Publisher("~result", AudioBuffer, queue_size=1)
        rospy.loginfo("... done")

    @staticmethod
    def to_radians(angle):
        return angle * np.pi / 180

    @staticmethod
    def get_length_of_vector(vector):
        return np.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)

    @staticmethod
    def to_unit_vector(azimuth, elevation):
        res = np.array([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
        l = DsBeamformer.get_length_of_vector(res)
        print "Unit vector %f." % l
        return res

    @staticmethod
    def compute_delay(mic, source):
        return np.dot(source, mic) / SPEED_OF_SOUND

    @staticmethod
    def delay_from_furthest_mic(delays):
        tmp = np.array(delays.values())
        min_delay, furthest_mic = np.min(tmp), np.argmin(tmp)
        tmp -= min_delay
        return dict(zip(delays.keys(), tmp)) #, min_delay, furthest_mic

    @staticmethod
    def time_to_samples(delays, sampling_rate):
        tmp = np.array(delays.values())*sampling_rate
        return dict(zip(delays.keys(), tmp))

    def callback(self, msg):
        self.channel_map = np.frombuffer(msg.channelMap, dtype=np.int8)
        self.frequency = msg.frequency
        self.chunk_size = len(msg.data)
        delays = DsBeamformer.compute_delays(self.channel_map, self.source, self.frequency)
        max_delay = np.max(delays.values())
        padding = dict(zip(delays.keys(), np.sqrt((np.array(delays.values(), dtype=np.uint) - max_delay)**2)))
        print delays, padding
        data = np.array(msg.data, dtype=np.int16).reshape((-1, len(mics)))
        for i, k in enumerate(self.channel_map):
            # self._buffer[k].put(data[:, i])
            self._buffer[k].put([0]*delays[k])
            self._buffer[k].put(data[:, i])
            self._buffer[k].put([0]*padding[k])

    @staticmethod
    def compute_delays(channel_map, source, sampling_rate):
        delays = {}
        for k in channel_map:
            delays[k] = DsBeamformer.compute_delay(mics[k], source)
        delays = DsBeamformer.time_to_samples(DsBeamformer.delay_from_furthest_mic(delays), sampling_rate)

        # Ignoring fractional delays for now
        delays = dict(zip(delays.keys(), np.round(np.array(delays.values()))))
        return delays

    def generator(self):
        chunk_size = 1024

        def get_buffer(size=1, block=True):
            res = []
            for k in self.channel_map:
                res.append(self._buffer[k].get(size=size, block=block))
            return res

        while not rospy.is_shutdown():
            # Use a blocking get() to ensure there's at least one chunk of
            # data, and stop iteration if the chunk is None, indicating the
            # end of the audio stream.
            data = np.sum(get_buffer(size=chunk_size), axis=0, dtype=np.int)/len(mics)
            data = data.tolist()

            # Now consume whatever other data's still buffered.
            while not rospy.is_shutdown(): # and len(data) != self.chunk_size:
                try:
                    chunk = np.sum(get_buffer(size=chunk_size, block=False), axis=0, dtype=np.int)/len(mics)
                    data.extend(chunk.tolist())
                except queue.Empty:
                    break

            yield data

    def spin(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            if self.channel_map is not None:
                a = AudioBuffer(
                    frequency=self.frequency,
                    channelMap=[0]
                )
                # a.data.extend(np.ravel(np.column_stack((x for ))).tolist())
                for data in self.generator():
                    a.header.stamp = rospy.Time.now()
                    a.data = data
                    self.pub.publish(a)
                    r.sleep()
            rospy.sleep(.01)


if __name__ == "__main__":
    rospy.init_node("mummer_ds_beamforming")
    d = DsBeamformer(rospy.get_name())
    d.spin()
