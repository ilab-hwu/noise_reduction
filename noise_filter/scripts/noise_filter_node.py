#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
import sounddevice as sd
import numpy as np
import librosa
from pysndfx import AudioEffectsChain
import math
import python_speech_features
import scipy as sp
from scipy import signal


class AudioStream(object):
    def __init__(self, name):
        rospy.loginfo("Starting {}.".format(name))
        self.stream = sd.OutputStream(48000, channels=1, dtype=np.int16)
        # self.stream = sd.OutputStream(48000, channels=4, dtype=np.int16)
        self.pub = rospy.Publisher("~result", AudioBuffer)
        rospy.Subscriber("/naoqi_driver_node/audio", AudioBuffer, self.callback)
        rospy.loginfo("done")

    '''------------------------------------
    NOISE REDUCTION USING POWER:
        receives an audio matrix,
        returns the matrix after gain reduction on noise
    ------------------------------------'''

    def reduce_noise_power(self, y, sr):

        cent = librosa.feature.spectral_centroid(y=y, sr=sr)

        threshold_h = round(np.median(cent)) * 1.5
        threshold_l = round(np.median(cent)) * 0.1

        less_noise = AudioEffectsChain().lowshelf(gain=-30.0, frequency=threshold_l, slope=0.8).highshelf(gain=-12.0,
                                                                                                          frequency=threshold_h,
                                                                                                          slope=0.5)  # .limiter(gain=6.0)
        y_clean = less_noise(y)

        return y_clean

    '''------------------------------------
    NOISE REDUCTION USING CENTROID ANALYSIS:
        receives an audio matrix,
        returns the matrix after gain reduction on noise
    ------------------------------------'''

    def reduce_noise_centroid_s(self, y, sr):

        cent = librosa.feature.spectral_centroid(y=y, sr=sr)

        threshold_h = np.max(cent)
        threshold_l = np.min(cent)

        less_noise = AudioEffectsChain().lowshelf(gain=-12.0, frequency=threshold_l, slope=0.5).highshelf(gain=-12.0,
                                                                                                          frequency=threshold_h,
                                                                                                          slope=0.5).limiter(
            gain=6.0)

        y_cleaned = less_noise(y)

        return y_cleaned

    def reduce_noise_centroid_mb(self, y, sr):

        cent = librosa.feature.spectral_centroid(y=y, sr=sr)

        threshold_h = np.max(cent)
        threshold_l = np.min(cent)

        less_noise = AudioEffectsChain().lowshelf(gain=-30.0, frequency=threshold_l, slope=0.5).highshelf(gain=-30.0,
                                                                                                          frequency=threshold_h,
                                                                                                          slope=0.5).limiter(
            gain=10.0)
        # less_noise = AudioEffectsChain().lowpass(frequency=threshold_h).highpass(frequency=threshold_l)
        y_cleaned = less_noise(y)

        cent_cleaned = librosa.feature.spectral_centroid(y=y_cleaned, sr=sr)
        columns, rows = cent_cleaned.shape
        boost_h = math.floor(rows / 3 * 2)
        boost_l = math.floor(rows / 6)
        boost = math.floor(rows / 3)

        # boost_bass = AudioEffectsChain().lowshelf(gain=20.0, frequency=boost, slope=0.8)
        boost_bass = AudioEffectsChain().lowshelf(gain=16.0, frequency=boost_h,
                                                  slope=0.5)  # .lowshelf(gain=-20.0, frequency=boost_l, slope=0.8)
        y_clean_boosted = boost_bass(y_cleaned)

        return y_clean_boosted

    '''------------------------------------
    NOISE REDUCTION USING MFCC:
        receives an audio matrix,
        returns the matrix after gain reduction on noise
    ------------------------------------'''

    def reduce_noise_mfcc_down(self, y, sr):

        hop_length = 512

        ## librosa
        # mfcc = librosa.feature.mfcc(y=y, sr=sr, hop_length=hop_length, n_mfcc=13)
        # librosa.mel_to_hz(mfcc)

        ## mfcc
        mfcc = python_speech_features.base.mfcc(y)
        mfcc = python_speech_features.base.logfbank(y)
        mfcc = python_speech_features.base.lifter(mfcc)

        sum_of_squares = []
        index = -1
        for r in mfcc:
            sum_of_squares.append(0)
            index = index + 1
            for n in r:
                sum_of_squares[index] = sum_of_squares[index] + n ** 2

        strongest_frame = sum_of_squares.index(max(sum_of_squares))
        hz = python_speech_features.base.mel2hz(mfcc[strongest_frame])

        max_hz = max(hz)
        min_hz = min(hz)

        speech_booster = AudioEffectsChain().highshelf(frequency=min_hz * (-1) * 1.2, gain=-12.0, slope=0.6).limiter(
            gain=8.0)
        y_speach_boosted = speech_booster(y)

        return (y_speach_boosted)

    def reduce_noise_mfcc_up(self, y, sr):

        hop_length = 512

        ## librosa
        # mfcc = librosa.feature.mfcc(y=y, sr=sr, hop_length=hop_length, n_mfcc=13)
        # librosa.mel_to_hz(mfcc)

        ## mfcc
        mfcc = python_speech_features.base.mfcc(y)
        mfcc = python_speech_features.base.logfbank(y)
        mfcc = python_speech_features.base.lifter(mfcc)

        sum_of_squares = []
        index = -1
        for r in mfcc:
            sum_of_squares.append(0)
            index = index + 1
            for n in r:
                sum_of_squares[index] = sum_of_squares[index] + n ** 2

        strongest_frame = sum_of_squares.index(max(sum_of_squares))
        hz = python_speech_features.base.mel2hz(mfcc[strongest_frame])

        max_hz = max(hz)
        min_hz = min(hz)

        speech_booster = AudioEffectsChain().lowshelf(frequency=min_hz * (-1), gain=12.0,
                                                      slope=0.5)  # .highshelf(frequency=min_hz*(-1)*1.2, gain=-12.0, slope=0.5)#.limiter(gain=8.0)
        y_speach_boosted = speech_booster(y)

        return (y_speach_boosted)

    '''------------------------------------
    NOISE REDUCTION USING MEDIAN:
        receives an audio matrix,
        returns the matrix after gain reduction on noise
    ------------------------------------'''

    def reduce_noise_median(self, y, sr):
        y = sp.signal.medfilt(y, 3)
        return (y)

    def callback(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ': %d samples received, freq = %d, channels = %d', len(msg.data),
                      msg.frequency, len(msg.channelMap))
        print np.fromstring(msg.channelMap, dtype='>u4')

        data = np.array(msg.data[0::4], dtype=np.int16, order='C')

        # data = np.array(msg.data, dtype=np.int16).reshape((-1, 4))

        start = rospy.Time.now().to_sec()
        print data.shape, rospy.Time.now().to_sec() - start
        a = AudioBuffer()
        a.header = msg.header
        a.frequency = msg.frequency
        a.channelMap = [0]
        a.data = self.reduce_noise_centroid_mb(data, msg.frequency)
        self.pub.publish(a)

if __name__ == "__main__":
    rospy.init_node("noise_filter_node")
    a = AudioStream(rospy.get_name())
    rospy.spin()