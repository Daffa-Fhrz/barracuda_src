#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Int32, Float32
from barracuda_roscom.msg import currentPose


class GlobalOdometry:
    def __init__(self):
        self.ppr = 120
        self.R = 0.03
        self.cos45 = 0.707106781186547524
        self.sin45 = 0.707106781186547524

        self.initial_encoder1 = None
        self.initial_encoder2 = None
        self.r1_prev = 0.0
        self.r2_prev = 0.0

        self.theta = 0.0
        self.x_global = 0.0
        self.y_global = 0.0

        self.currentPosePub = rospy.Publisher('/barracuda_kinematic/odometry/global/pose', currentPose, queue_size=10)

    def callback_heading(self, data):
        self.theta = data.data

    def callback_encoder1(self, data):
        if self.initial_encoder1 is None:
            self.initial_encoder1 = data.data
        r1 = ((data.data - self.initial_encoder1) / self.ppr) * (2 * np.pi * self.R)
        self.update(r1=r1)

    def callback_encoder2(self, data):
        if self.initial_encoder2 is None:
            self.initial_encoder2 = data.data
        r2 = ((data.data - self.initial_encoder2) / self.ppr) * (2 * np.pi * self.R)
        self.update(r2=r2)

    def update(self, r1=None, r2=None):
        r1 = r1 if r1 is not None else self.r1_prev
        r2 = r2 if r2 is not None else self.r2_prev

        delta_r1 = r1 - self.r1_prev
        delta_r2 = r2 - self.r2_prev
        self.r1_prev = r1
        self.r2_prev = r2

        dx_local = 0.5 * delta_r1 * self.cos45 + 0.5 * delta_r2 * self.sin45
        dy_local = 0.5 * delta_r1 * (-self.sin45) + 0.5 * delta_r2 * self.cos45

        theta = self.theta
        dx_global = dx_local * np.cos(theta) - dy_local * np.sin(theta)
        dy_global = dx_local * np.sin(theta) + dy_local * np.cos(theta)

        self.x_global += dx_global
        self.y_global += dy_global

        self.publish()

    def publish(self):
        posisi = currentPose()
        posisi.x = self.x_global
        posisi.y = self.y_global
        self.currentPosePub.publish(posisi)


if __name__ == '__main__':
    rospy.init_node('global_odometry')
    node = GlobalOdometry()
    rospy.Subscriber('/arduino/encEksternal/counter1', Int32, node.callback_encoder1)
    rospy.Subscriber('/arduino/encEksternal/counter2', Int32, node.callback_encoder2)
    rospy.Subscriber('/arduino/theta/heading', Float32, node.callback_heading)
    rospy.spin()
