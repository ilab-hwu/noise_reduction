#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from naoqi_bridge_msgs.msg import AudioBuffer
import numpy as np
import librosa
from pysndfx import AudioEffectsChain
import math
import python_speech_features
import scipy as sp
import adaptfilt as af
import yaml
import padasip as pa
import scipy.fftpack as fftpack
from noise_filter.cfg import NoiseFilterConfig
from dynamic_reconfigure.server import Server as DynServer


# Most of the functions are taken from https://github.com/dodiku/noise_reduction


class AudioStream(object):
    def __init__(self, name):
        rospy.loginfo("Starting {}.".format(name))

        self.functions = {
            "none": lambda y, _: y,
            "power": self.reduce_noise_power,
            "centroid_s": self.reduce_noise_centroid_s,
            "centroid_mb": self.reduce_noise_centroid_mb,
            "mfcc_down": self.reduce_noise_mfcc_down,
            "mfcc_up": self.reduce_noise_mfcc_up,
            "median": self.reduce_noise_median
        }

        self.dyn_srv = DynServer(NoiseFilterConfig, self.dyn_callback)
        self.pub = rospy.Publisher("~result", AudioBuffer, queue_size=1)
        # with open('/home/cd32/noise.yaml', 'r') as f:
        #     self.noise = np.array(yaml.load(f)).reshape(-1, 4)
        rospy.Subscriber("/mummer_ds_beamforming/result", AudioBuffer, self.callback)
        # rospy.Subscriber("/naoqi_driver_node/audio", AudioBuffer, self.callback)
        # print self.noise.shape
        rospy.loginfo("done")

    def dyn_callback(self, config, level):
        self.func_order = {}
        for k, v in config.items():
            key = k.split('_')
            if key[0] == "Function":
                self.func_order[int(key[1])] = v
        return config

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

    def least_mean_squares(self, y, sr):
        # noise = self.noise[:, 0].astype(float)/np.iinfo(np.int16).max
        # y = y.astype(float)/np.iinfo(np.int16).max
        # print "NOISE", noise
        # print "INPUT", y
        # w, _, _ = af.lms(noise, y, 20, 0.03)
        # w *= np.iinfo(np.int16).max
        # print "CLEAN", w
        # return w

        n = 20
        x = pa.input_from_history(y, n)[:-1]
        print x
        print y.shape
        # y = y[n:]
        f = pa.filters.FilterRLS(mu=0.9, n=n)
        y, e, w = f.run(y, x)
        return y

    def subtract_noise(self, y, sr):
        return np.round(np.fft.irfft(np.fft.rfft(y)/np.fft.rfft(self.noise[:, 0]))).astype(np.int16)


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
        rospy.logdebug(rospy.get_caller_id() + ': %d samples received, freq = %d, channels = %d', len(msg.data),
                      msg.frequency, len(msg.channelMap))
        data = np.array(msg.data, dtype=np.int16, order='C')

        for i in range(len(self.func_order.keys())):
            data = self.functions[self.func_order[i]](data, msg.frequency)

        self.pub.publish(
            AudioBuffer(
                header=msg.header,
                frequency=msg.frequency,
                channelMap=msg.channelMap,
                data=data
            )
        )

if __name__ == "__main__":
    rospy.init_node("noise_filter_node")
    a = AudioStream(rospy.get_name())
    rospy.spin()
