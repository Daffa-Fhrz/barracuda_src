#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from encoder_subscriber.libencoder import baca_encoder

encoder1_data = 0
encoder2_data = 0
initial_encoder1 = None
initial_encoder2 = None

def callback_encoder1(data):
    global encoder1_data, initial_encoder1
    if initial_encoder1 is None:
        initial_encoder1 = data.data
    encoder1_data = data.data - initial_encoder1
    print_output()

def callback_encoder2(data):
    global encoder2_data, initial_encoder2
    if initial_encoder2 is None:
        initial_encoder2 = data.data
    encoder2_data = data.data - initial_encoder2
    print_output()

def print_output():
    enc = baca_encoder()
    xencoder, yencoder = enc.perumusan(encoder1_data, encoder2_data)
    rospy.loginfo(f"Data X : {xencoder}\tData Y : {yencoder}")

def listener():
    global encoder1_data, encoder2_data, initial_encoder1, initial_encoder2, xencoder, yencoder

    encoder1_data = 0
    encoder2_data = 0
    initial_encoder1 = None
    initial_encoder2 = None

    rospy.init_node('encoder_subscriber', anonymous=True)

    rospy.Subscriber("encoder1_data", Int32, callback_encoder1)
    rospy.Subscriber("encoder2_data", Int32, callback_encoder2)
    
    rospy.spin()

if __name__ == '__main__':
    listener()