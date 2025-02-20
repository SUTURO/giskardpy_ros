#!/usr/bin/env python

import string

import geometry_msgs
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, lfilter
from collections import deque


class Job:
    def do_work(self, data: deque) -> np.ndarray:
        pass


class Filter(Job):
    def __init__(self, cutoff, fs, order):
        self.cutoff = cutoff
        self.fs = fs
        self.order = order

    def do_work(self, data: deque):
        return self.butter_lowpass_filter(data, self.cutoff, self.fs, self.order)

    def butter_lowpass(self, cutoff, fs, order=5):
        nyq = 0.5 * fs  # Nyquist Frequency
        normal_cutoff = cutoff / nyq
        return butter(order, normal_cutoff, btype='low', analog=False)

    def butter_lowpass_filter(self, data: deque, cutoff, fs, order=5) -> np.ndarray:
        """
        The Lowpass Filter. used to filter out signal noise. For more info on how the lowpass filters works see:
        https://en.wikipedia.org/wiki/Low-pass_filter
        """
        b, a = self.butter_lowpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y


class Diff(Job):
    def __init__(self, dt: float):
        self.dt = dt

    def do_work(self, data: deque) -> np.ndarray:
        """
        The gradient filter, puts out a gradient of the signal it's filtering
        """
        return np.gradient(data) / self.dt


class ForceTorqueFilter:

    def __init__(self,
                 input_topic: str,
                 output_topic: str,
                 worker: Job):
        """
        The ForceTorqueRawFilter is a filter that takes the signal published by both /hsrb/wrist_wrench/compensated
        and /hsrb/wrist_wrench/raw and applies filters to them (gradient to /hsrb/wrist_wrench/compensated and
        low pass filter + gradient to /hsrb/wrist_wrench/raw in that order).
        These filtered values are then put in queues (one for each value) and are published onto
        their own topics, compensated/diff and filtered_raw/diff respectively.

        :param input_topic: the topic from which the original signal is taken
        :param output_topic: the topic the filtered signal is published to
        :param worker: the filter that is being applied to the signal
        """
        # TODO: make queue size dynamic to sample rate
        self.trans_F_x = deque(maxlen=100 * 2)
        self.trans_F_y = deque(maxlen=100 * 2)
        self.trans_F_z = deque(maxlen=100 * 2)
        self.trans_T_x = deque(maxlen=100 * 2)
        self.trans_T_y = deque(maxlen=100 * 2)
        self.trans_T_z = deque(maxlen=100 * 2)
        self.worker = worker
        self.topic = input_topic
        self.wrench = WrenchStamped()
        self.subscriber = rospy.Subscriber(name=self.topic,
                                           data_class=WrenchStamped, callback=self.cb)

        self.pub = rospy.Publisher(name=output_topic, data_class=WrenchStamped, queue_size=10)
        self.rate = rospy.Rate(10)

    def cb(self, rawdata: WrenchStamped):
        self.wrench = rawdata

        # Add values into respective queues
        self.trans_F_x.append(self.wrench.wrench.force.x)
        self.trans_F_y.append(self.wrench.wrench.force.y)
        self.trans_F_z.append(self.wrench.wrench.force.z)
        self.trans_T_x.append(self.wrench.wrench.torque.x)
        self.trans_T_y.append(self.wrench.wrench.torque.y)
        self.trans_T_z.append(self.wrench.wrench.torque.z)

        # Filter the data
        if len(self.trans_F_x) < 10:
            return
        filtered_data = self.worker.do_work(self.trans_F_x)
        filtered_data2 = self.worker.do_work(self.trans_F_y)
        filtered_data3 = self.worker.do_work(self.trans_F_z)
        filtered_data4 = self.worker.do_work(self.trans_T_x)
        filtered_data5 = self.worker.do_work(self.trans_T_y)
        filtered_data6 = self.worker.do_work(self.trans_T_z)
        # publish signal
        self.pub.publish(
            self.rebuild_signal(filtered_data[-1], filtered_data2[-1], filtered_data3[-1], filtered_data4[-1],
                                filtered_data5[-1],
                                filtered_data6[-1]))
        # rebuilding signal for publishing

    def rebuild_signal(self, F_x: np.ndarray, F_y: np.ndarray, F_z: np.ndarray,
                       T_x: np.ndarray, T_y: np.ndarray, T_z: np.ndarray) -> WrenchStamped:
        """
        Rebuilds the signal with the filtered values back into a WrenchStamped

        :param F_x: X-value of the Force of the filtered signal
        :param F_y: Y-value of the Force of the filtered signal
        :param F_z: Z-value of the Force of the filtered signal
        :param T_x: X-value of the Torque of the filtered signal
        :param T_y: Y-value of the Torque of the filtered signal
        :param T_z: Z-value of the Torque of the filtered signal

        :return: WrenchStamped message
        """

        rebuild_force = geometry_msgs.msg.Vector3(F_x, F_y, F_z)
        rebuild_torque = geometry_msgs.msg.Vector3(T_x, T_y, T_z)

        rebuild_wrench = geometry_msgs.msg.Wrench(rebuild_force, rebuild_torque)

        rebuild_wrenchStamped = geometry_msgs.msg.WrenchStamped(self.wrench.header, rebuild_wrench)

        return rebuild_wrenchStamped


if __name__ == '__main__':
    rospy.init_node('force_torque_raw_filter')
    force = ForceTorqueFilter(input_topic="/hsrb/wrist_wrench/raw",
                              output_topic='filtered_raw',
                              worker=Filter(fs=100,  # Sample rate, Hz, currently set to Hz of the topic
                                            cutoff=5,  # Desired cutoff frequency of the filter, Hz
                                            order=5))  # Order of the filter
    diff = ForceTorqueFilter(input_topic="/hsrb/wrist_wrench/compensated",
                             output_topic='compensated/diff',
                             worker=Diff(dt=0.01))  # Order of the filter
    force_diff = ForceTorqueFilter(input_topic="filtered_raw",
                                   output_topic='filtered_raw/diff',
                                   worker=Diff(dt=0.01))  # Order of the filter
    rospy.loginfo('wrench filter running')
    rospy.spin()
