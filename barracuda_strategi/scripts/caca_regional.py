#!/usr/bin/env python3
import rospy
from time import sleep
from enum import Enum
import math
# from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from geometry_msgs.msg import Point, Pose2D, Twist, Quaternion


class Enable(Enum):
    Stop = 0
    moda1 = 1
    moda2 = 2
    moda3 = 3
    moda4 = 4
    moda5 = 5

    ## NASIONAL
	# Stop = 0
	# OnPlay = 1
	# StartPosition = 2
	# DropBall = 3
	# KickOffHome = 4
	# KickOffAway = 5
	# GoalKickHome = 6
	# GoalKickAway = 7
	# FreeKickHome = 8
	# FreeKickAway = 9
	# CornerHome = 10
	# CornerAway = 11
	# PenaltyHome = 12
	# PenaltyAway = 13
	# ThrowInHome = 14
	# ThrowInAway = 15
	# Empty = 16
	# # Additional
	# ResetOdometry = 17
	# KickBall = 18

class RobotState(Enum):
    Stop = 0
    GotoAbsPose = 1
    GotoBall = 2
    DribleBall = 3
    WaitBall = 4
    WaitFriend = 5

# Initialization of variables
enable = Enable.Stop
cici_pose = Pose2D()
cici_state = RobotState.Stop
caca_pose = Pose2D()
ball_reached = False
front_ball_detected = False
front_ball_point = Quaternion()
omni_ball_detected = False
omni_ball_point = Quaternion()
front_dummy_detected = False
front_dummy_point = Quaternion()
omni_dummy_detected = False
omni_dummy_point = Quaternion()

dummy = [False, False, False, False, 
         False, False, False, False, 
         False, False, False, False]


## PUBLISHER
# Friend
caca_pose_pub = rospy.Publisher('/barra_komunikasi', Pose2D, queue_size=10)
caca_state_pub = rospy.Publisher('/barra_komunikasi', Byte, queue_size=10)
# Micro
robot_shoot_ball_pub = rospy.Publisher('/barra_arduino/shoot', Byte, queue_size=10)



#### FUNGSI FUNGSI






if __name__ == '__main__':
    rospy.init_node('caca_regional')
    
    # Subscriber
    # serial node
    rospy.Subscriber('/barra_arduino/ball_reached', Bool, BallReachedHandler)
	# computer vision node
    rospy.Subscriber('/barra_vision/front_cam/ball_detected', Bool, FrontBallDetectedHandler)
    rospy.Subscriber('/barra_vision/front_cam/ball_point', Quaternion, FrontBallPointHandler)
    rospy.Subscriber('/barra_vision/omni_cam/ball_detected', Bool, OmniBallDetectedHandler)
    rospy.Subscriber('/barra_vision/omni_cam/ball_point', Quaternion, OmniBallPointHandler)
    rospy.Subscriber('/barra_vision/front_cam/dummy_detected', Bool, FrontDummyDetectedHandler)
    rospy.Subscriber('/barra_vision/front_cam/dummy_point', Quaternion, FrontDummyPointHandler)
    rospy.Subscriber('/barra_vision/omni_cam/dummy_detected', Bool, OmniDummyDetectedHandler)
    rospy.Subscriber('/barra_vision/omni_cam/dummy_point', Quaternion, OmniDummyPointHandler)
    #  odometry node
    rospy.Subscriber('/barra_odo/pose', Pose2D, OdoHandler) 



    #### ALGORITMA MODE
    



