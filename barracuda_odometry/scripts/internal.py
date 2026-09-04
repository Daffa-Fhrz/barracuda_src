#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Float32
from barracuda_kinematic.encoder import Internal
from barracuda_kinematic.msg import currentPose     

class Input:
    def __init__(self):
        self.encoder1_data = 0
        self.encoder2_data = 0
        self.encoder3_data = 0
        self.encoder4_data = 0
        self.initial_encoder1 = None
        self.initial_encoder2 = None
        self.initial_encoder3 = None
        self.initial_encoder4 = None
        self.x = 0
        self.y = 0
        self.posisixPub = rospy.Publisher('/barracuda_kinematic/odometry/internal/pose', currentPose, queue_size=10)
    
    def callback_encoder1(self, data):
        if self.initial_encoder1 is None:
            self.initial_encoder1 = data.data
        self.encoder1_data = data.data - self.initial_encoder1
        self.Run()

    def callback_encoder2(self, data):
        if self.initial_encoder2 is None:
            self.initial_encoder2 = data.data
        self.encoder2_data = data.data - self.initial_encoder2
        self.Run()

    def callback_encoder3(self, data):
        if self.initial_encoder3 is None:
            self.initial_encoder3 = data.data
        self.encoder3_data = data.data - self.initial_encoder3
        self.Run()

    def callback_encoder4(self, data):
        if self.initial_encoder4 is None:
            self.initial_encoder4 = data.data
        self.encoder4_data = data.data - self.initial_encoder4
        self.Run()

    def Run(self):
        enc = Internal()
        self.x, self.y = enc.perumusan(self.encoder1_data, self.encoder2_data, self.encoder3_data, self.encoder4_data)
        posisi = currentPose()
        posisi.x = self.x * 454.8126940264768751
        posisi.y = self.y * 454.8126940264768751
        self.posisixPub.publish(posisi)

if __name__ == '__main__':
    rospy.init_node('encoder_subscriber', anonymous=True)
    inp = Input()
    rospy.Subscriber("/arduino/encoder/internal/counter1", Int32, inp.callback_encoder1)
    rospy.Subscriber("/arduino/encoder/internal/counter2", Int32, inp.callback_encoder2)
    rospy.Subscriber("/arduino/encoder/internal/counter3", Int32, inp.callback_encoder3)
    rospy.Subscriber("/arduino/encoder/internal/counter4", Int32, inp.callback_encoder4)
    rate = rospy.Rate(10)  
    while not rospy.is_shutdown():
        inp.Run()
        rate.sleep()