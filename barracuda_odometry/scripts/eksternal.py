#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from barracuda_odometry.rumus import Eksternal
from barracuda_roscom.msg import currentPose

class Input:
    def __init__(self):
        self.encoder1_data = 0
        self.encoder2_data = 0
        self.initial_encoder1 = None
        self.initial_encoder2 = None
        self.x = 0
        self.y = 0
        self.currentPosePub = rospy.Publisher('/barracuda_kinematic/odometry/eksternal/pose', currentPose, queue_size=10)
    def callback_encoder1(self, data):
        if self.initial_encoder1 is None:
            self.initial_encoder1 = data.data
        self.encoder1_data = data.data - self.initial_encoder1

    def callback_encoder2(self, data):
        if self.initial_encoder2 is None:
            self.initial_encoder2 = data.data
        self.encoder2_data = data.data - self.initial_encoder2

    def Run(self):
        enc = Eksternal()
        self.x, self.y = enc.perumusan(self.encoder1_data, self.encoder2_data)
        posisi = currentPose()
        posisi.x = self.x 
        posisi.y = self.y 
        self.currentPosePub.publish(posisi)

if __name__ == '__main__':
    rospy.init_node('encoder_subscriber', anonymous=True)
    inp = Input()
    rospy.Subscriber("/arduino/encEksternal/counter1", Int32, inp.callback_encoder1)
    rospy.Subscriber("/arduino/encEksternal/counter2", Int32, inp.callback_encoder2)
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        inp.Run()
        rate.sleep()