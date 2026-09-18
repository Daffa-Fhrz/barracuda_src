#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Pose2D
from barracuda_roscom.msg import currentPose
from std_msgs.msg import Float32
class Kinematic():
    def __init__(self):
        # --------- Pose
        self.odom = Pose2D()
        self.odomlokalx = 0
        self.odomlokaly = 0
        self.lokallamax = 0
        self.lokallamay = 0
        self.odom.x = 0
        self.odom.y = 0
        self.odom.theta = 0
        self.posePub = rospy.Publisher('/robot/kinematic/odometry/pose', Pose2D, queue_size=10)
        # --------- Tap to Move Command (TtMC)
        self.TtMCPub = rospy.Publisher('/barracuda_kinematic/TtMC/target', Pose2D, queue_size=10)
    
    def RobotPoseCallback(self, data, args):
        index = args
        # ------ xy dari currentPose
        if index == 0:
            self.odomlokalx = data.x * 57
            self.odomlokaly = data.y * 65
            # self.odom.x = data.x * 57
            # self.odom.y = data.y * 65
        # ------ theta dari BMM heading
        if index == 1:
            self.odom.theta = math.radians(360 - data.data)
        # Publish
        delta_x_lokal = self.odomlokalx - self.lokallamax
        delta_y_lokal = self.odomlokaly - self.lokallamay
        if (delta_x_lokal != 0.0) or (delta_y_lokal != 0.0):
            self.odom.x += ((delta_x_lokal * math.cos(self.odom.theta)) - (delta_y_lokal * math.sin(self.odom.theta))) #* -1
            self.odom.y += ((delta_x_lokal * math.sin(self.odom.theta)) + (delta_y_lokal * math.cos(self.odom.theta))) #* -1
            self.lokallamax = self.odomlokalx
            self.lokallamay = self.odomlokaly
        self.posePub.publish(self.odom)
    
    def BSTtMCCallback(self, data):
        self.TtMCPub.publish(data)




if __name__ == '__main__':
    rospy.init_node('robot_communicaton')
    #? Kinematic
    kine = Kinematic()
    #! --------- Pose
    rospy.Subscriber('/barracuda_kinematic/odometry/eksternal/pose', currentPose, kine.RobotPoseCallback, callback_args=0)
    rospy.Subscriber('/arduino/theta/heading', Float32, kine.RobotPoseCallback, callback_args=1)
    #! --------- Tap to Move Command (TtMC)
    rospy.Subscriber('/robot/pose', Pose2D, kine.BSTtMCCallback)
    # rospy.Subscriber('/robot/caca/theta', Float32, kine.BSTtheta)
    rospy.spin()