#!/usr/bin/env python3
import rospy
import numpy as np
from barracuda_roscom.msg import speedRobot , currentPose, penggiring, ballCatch
from yolo_barra.msg import (
    YoloResult,

    ballInfo,
    ballTravel,

    robotInfo,
    robotInfoArray,

    goalInfo,

    dummyInfo,
    dummyInfoArray
)
from std_msgs.msg import Bool, Int16, Int8
from geometry_msgs.msg import Pose2D
# from barracuda_kinematic.PID_controller import PIDController # type: ignore
from enum import Enum   

class robotState(Enum):
    STOP = 0
    GOTO_ABSPOSE = 1
    GOTO_BALL = 2
    DRIBLE_BALL = 3
    WAIT_BALL = 4
    WAIT_FRIEND = 5
    PRESSING = 6
    NOBALLS = 7


targetPosex = 0
targetPosey =0
targetPosetheta = 0
currentPosex = 0
currentPosey = 0

ball_point_omni = ballInfo()
ball_pos_omni = ballTravel()
ballStatus_omni = False

ball_point_front = ballInfo()
ball_pos_front = ballTravel()
ballStatus_front = False

targetPose = Pose2D()
poseNow = currentPose()
proximityState = ballCatch()         #PROXIMITY SENSOR   

command = robotState.STOP.value

headingBMM = 0
headingBNO = 0

speed = speedRobot()    
speedPenggiring = penggiring()

#PUBLISHER SETUP    
wheellSpeed_pub = rospy.Publisher('/barracuda_kinematic/wheel/speed', speedRobot, queue_size=10)   
penggiring_pub = rospy.Publisher('/barracuda_kinematic/penggiring/speed', penggiring, queue_size=3)

class PIDController:
    def __init__(self, Kp, Ki, Kd, MaxValue, MinValue, T=0.02, MaxValue_Integer=2, MinValue_integer=-2, SamplingTime=0.01, limit_EN=True):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = T
        self.limMin = MinValue
        self.limMax = MaxValue
        self.limMin_Int = MinValue_integer
        self.limMax_int = MaxValue_Integer
        self.TimeSam = SamplingTime
        self.integrator = 0
        self.prevError = 0
        self.differentiator = 0
        self.prevMeasurement = 0
        self.outVal = 0
        self.limit_EN = limit_EN

    def PID_Calc(self, setPoint, measurement):
        error = setPoint - measurement
        propotional = self.Kp * error
        self.integrator = self.integrator + 0.5 * self.Ki * self.TimeSam * (error + self.prevError)

        if self.integrator > self.limMax_int:
            self.integrator = self.limMax_int
        elif self.integrator < self.limMin_Int:
            self.integrator = self.limMin_Int

        self.differentiator = -(2.0 * self.Kd * (measurement - self.prevMeasurement)
                                + (2.0 * self.tau - self.TimeSam) * self.differentiator) \
                               / (2.0 * self.tau + self.TimeSam)

        self.outVal = propotional + self.integrator + self.differentiator

        if self.limit_EN:
            if self.outVal > self.limMax:
                self.outVal = self.limMax
            elif self.outVal < self.limMin:
                self.outVal = self.limMin

        self.prevError = error
        self.prevMeasurement = measurement

        return int(self.outVal)  # Konversi ke integer


class IKMove:
    def __init__(self):
        self.r = 5.2  # Radius dalam cm
        self.L = 24  # Jarak pusat Robot - Roda cm
        # self.pwm_max = 130
        self.xy_max = 1870
        self.theta_max = 55
        self.cossin45 = 0.70710678118654752
        #   ---OMNI CAMERA--- (UNIVERSAL THETA)
        self.PID_angle_omni = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.075,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_x_omni = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )   
        self.PID_y_omni = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )  
        #   ---FRONT CAMERA---
        self.PID_angle_front = PIDController(
            Kp=0.085, Ki=0.000000000032, Kd=0.0077,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_distance = PIDController(
            Kp=4, Ki=0.0, Kd=0.01,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
            limit_EN=True
        )
        #   ---BMM- --
        self.PID_BMM = PIDController(
            Kp=1.5, Ki=0.0, Kd=0.012,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        #   ---BNO---
        self.PID_BNO = PIDController(
            Kp=1.5, Ki=0.0, Kd=0.012,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        # ---BALL XY--- 
        self.PID_ball_x = PIDController(    
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_ball_y = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        # ---DUMMY XY---   
        self.PID_dummy_x = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_dummy_y = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        # --- GAWANG XY ---
        self.PID_gawang_x = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True   
        )
        self.PID_gawang_y = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        # --- TARGET POSITION XYTHETA ---
        self.PID_target_x = PIDController(
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_target_y = PIDController(  
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        self.PID_target_theta = PIDController(          
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
            
    

    # FUNCTION SECTION
    def perumusan(self, vx, vy, vtheta, max_speed):
        vy = -vy
        # vx = -vx
        pos_array = [vx, vy]
        long_pos = max(abs(pos) for pos in pos_array)

        if long_pos > self.xy_max:
            vx = int(vx / long_pos * self.xy_max)
            vy = int((vy / long_pos * self.xy_max))

        vtheta = -1 * int(np.clip(vtheta, -self.theta_max, self.theta_max))

        w1 = int((vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r)
        w2 = int((-vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r)
        w3 = int((-vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r)
        w4 = int((vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r)

        speed_array = [w1, w2, w3, w4]
        hgh_speed = max(abs(speed) for speed in speed_array)

        if hgh_speed > max_speed:
            w1 = int(w1 / hgh_speed * max_speed)
            w2 = int(w2 / hgh_speed * max_speed)
            w3 = int(w3 / hgh_speed * max_speed)
            w4 = int(w4 / hgh_speed * max_speed)

        return w1, w2, w3, w4
    


# CALLBACK SECTION
def bno_heading(data):
    global headingBNO
    headingBNO = data.data 
    #((data.data + 180) % 360) - 180 (RUMUS KONVERSI BNO)
def bmm_heading(data):
    global headingBMM
    headingBMM = data.data
def ballStatus_cb_front(data):
    global ballStatus_front
    ballStatus_front = data.data
def ballStatus_cb_omni(data):
    global ballStatus_omni
    ballStatus_omni = data.data
def ballInfo_cb_front(data):
    global ball_point_front
    if ballStatus_front:
        ball_point_front = data
    else:
        ball_point_front.x = 0
        ball_point_front.y = 0
        ball_point_front.radius = 0
def ballInfo_cb_omni(data):
    global ball_point_omni
    if ballStatus_omni:
        ball_point_omni = data
    else:
        ball_point_omni.x = 0
        ball_point_omni.y = 0
        ball_point_omni.radius = 0
def ballTravel_cb_front(data):
    global ball_pos_front
    if ballStatus_front:
        ball_pos_front = data
    else:
        ball_pos_front.distance = 0
        ball_pos_front.degree = 0
def ballTravel_cb_omni(data):
    global ball_pos_omni
    if ballStatus_omni:
        ball_pos_omni = data
    else:
        ball_pos_omni.distance = 0
        ball_pos_omni.degree = 0
def strategy_cb(data):
    global targetPose
    targetPose.x = data.x
    targetPose.y = data.y
    targetPose.theta = data.theta
def current_pose_cb(data):
    global currentPosex, currentPosey
    currentPosex = data.x
    currentPosey = data.y
def proximity_cb(data):
    global ballCatch
    ballCatch.ballCatch1 = data.ballCatch1
    ballCatch.ballCatch2 = data.ballCatch2
def command_cb(data):
    global command
    command = data.data
def TtMC_cb(data):
    global targetPosex, targetPosey, targetPosetheta
    targetPosex = data.x
    targetPosey = data.y
    targetPosetheta = data.theta
def encoder_cb(data):
    global encoderx, encodery
    encoderx = data.x
    encodery = data.y
#---FUNCTION ROBOT STATE---
iK_move = IKMove()  

def penggiringSpeed(w1, w2):    
    global speedPenggiring
    speedPenggiring.Drible1 = w1
    speedPenggiring.Drible2 = w2
    penggiring_pub.publish(speedPenggiring)

def stopState():
    global speed
    speed.w1 = 0
    speed.w2 = 0
    speed.w3 = 0
    speed.w4 = 0
    wheellSpeed_pub.publish(speed)
    
def absPoseState():
    global speed
    iK_move.x_abs = iK_move.PID_target_x.PID_Calc(iK_move.target_x, iK_move.current_x)    
    iK_move.y_abs = iK_move.PID_target_y.PID_Calc(iK_move.target_y, iK_move.current_y)
    iK_move.theta_abs = iK_move.PID_target_theta.PID_Calc(iK_move.target_theta, iK_move.headingBNO)
    w1, w2, w3, w4 = iK_move.perumusan(iK_move.x_abs, iK_move.y_abs, iK_move.theta_abs, 200)
    speed.w1 = w1
    speed.w2 = w2
    speed.w3 = w3
    speed.w4 = w4
    wheellSpeed_pub.publish(speed)

def goToBallState_yTheta():
    global ballStatus_front, ball_pos_front, ballStatus_omni, ball_pos_omni, command, speed,proximityState
    if ballStatus_front:
        distance_now = ball_pos_front.distance 
        angle_now = ball_pos_front.degree
        distance = iK_move.PID_distance.PID_Calc(30, distance_now)
        angle = iK_move.PID_angle_front.PID_Calc(90, angle_now)
        w1, w2, w3, w4 = iK_move.perumusan(0, distance * 5, angle, 130)
        speed.w1 = w1
        speed.w2 = w2
        speed.w3 = w3
        speed.w4 = w4   
        wheellSpeed_pub.publish(speed)
        # penggiringSpeed(100, 100)
        # if proximityState.ballCatch1 or proximityState.ballCatch2:
        #     command = robotState.DRIBLE_BALL
    elif ballStatus_omni:
        angle_now = ((ball_pos_omni.degree + 180) % 360) - 180
        angle = iK_move.PID_angle_omni.PID_Calc(90, angle_now)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 130)
        speed.w1 = w1
        speed.w2 = w2
        speed.w3 = w3
        speed.w4 = w4
        wheellSpeed_pub.publish(speed)
    else:
        speed.w1 = 0
        speed.w2 = 0    
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)

def dribleState():
    global proximityState
    if proximityState.ballCatch1 or proximityState.ballCatch2:
        pass

def waitBallState():
    global command
    command = robotState.WAIT_BALL

def waitFriendState():
    global command
    command = robotState.WAIT_FRIEND

def pressingState():
    global command
    command = robotState.PRESSING

def noBallsState():
    global command
    command = robotState.NOBALLS 
    

#MAIN PROGRAM
def runProgram():
    global command
    if command == robotState.STOP.value:
        rospy.loginfo("STOP STATE")
        stopState() 
    
    elif command == robotState.GOTO_ABSPOSE.value:
        rospy.loginfo("GOTO_ABSPOSE")

    elif command == robotState.GOTO_BALL.value:
        rospy.loginfo("GOTO_BALL")
        goToBallState_yTheta()

def mainRun():
    if ballStatus_omni:
        angle_now = ball_pos_omni.degree
        angle = iK_move.PID_angle_omni.PID_Calc(90, angle_now)
        # if self.ballStatus_front:
        #     distance_now = self.ball_pos_front.distance
        #     distance = self.PID_distance.PID_Calc(10, distance_now)
        # else:
        #     distance = 0
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 40)
        speed.w1 = -w1
        speed.w2 = -w2
        speed.w3 = -w3
        speed.w4 = -w4
        wheellSpeed_pub.publish(speed)
    else:
        speed.w1 = 0
        speed.w2 = 0
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)

def mainRunV2():
    if ballStatus_front:
        distance_now = ball_pos_front.distance
        angle_now = ball_pos_front.degree
        distance = iK_move.PID_distance.PID_Calc(40, distance_now)
        angle = iK_move.PID_angle_front.PID_Calc(90, angle_now)
        w1, w2, w3, w4 = iK_move.perumusan(0, distance*10, angle, 40)
        speed.w1 = -w1
        speed.w2 = -w2
        speed.w3 = -w3
        speed.w4 = -w4
        wheellSpeed_pub.publish(speed)
    elif ballStatus_omni:
        angle_now = ball_pos_omni.degree
        angle = iK_move.PID_angle_omni.PID_Calc(90, angle_now)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 40)
        speed.w1 = -w1
        speed.w2 = -w2
        speed.w3 = -w3
        speed.w4 = -w4
        wheellSpeed_pub.publish(speed)
    else:
        speed.w1 = 0
        speed.w2 = 0
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)

def RunTtMC():
    distancex = targetPosex
    distancey = targetPosey
    x = iK_move.PID_distance.PID_Calc(distancex, currentPosex * 70)
    y = iK_move.PID_distance.PID_Calc(distancey, currentPosey * 70)
    w1, w2, w3, w4 = iK_move.perumusan(x, y, 0, 35)
    speed.w1 = w1
    speed.w2 = w2
    speed.w3 = w3
    speed.w4 = w4   
    wheellSpeed_pub.publish(speed)
        


if __name__ == '__main__':
    rospy.init_node('IK_kinematic')
    # SUBSCRIBER SETUP
    # ---VISION---
    ballInfo_omni_get = rospy.Subscriber('/Barracuda_Yolo/omni/ballInfo', ballInfo, ballInfo_cb_omni)
    ballTravel_omni_get = rospy.Subscriber('/Barracuda_Yolo/omni/ballTravel', ballTravel, ballTravel_cb_omni)
    ballStatus_omni_get = rospy.Subscriber("/barracuda_vision/camera/omni/ballStatus", Bool, ballStatus_cb_omni)
    ballInfo_front_get = rospy.Subscriber('/Barracuda_Yolo/front/ballInfo', ballInfo, ballInfo_cb_front)
    ballTravel_front_get = rospy.Subscriber('/Barracuda_Yolo/front/ballTravel', ballTravel, ballTravel_cb_front)
    ballStatus_front_get = rospy.Subscriber("/barracuda_vision/camera/front/ballStatus", Bool, ballStatus_cb_front)
    # ---ARDUINO---
    headingBMM_get = rospy.Subscriber("/arduino/BMM/heading", Int16, bmm_heading)
    headingBNO_get = rospy.Subscriber("/arduino/BNO/heading", Int16, bno_heading)
    proximity_get = rospy.Subscriber("/arduino/proximity/ballCatch", ballCatch, proximity_cb)   
    # ---STRATEGY---
    strategy_get = rospy.Subscriber("/barracuda_strategy/strategy/targetPose", Pose2D, strategy_cb)
    #---CURRENT POSE---
    current_pose_get = rospy.Subscriber("/barracuda_kinematic/odometry/eksternal/pose", currentPose, current_pose_cb)    
    #---COMMAND---
    command_get = rospy.Subscriber("/barracuda_strategy/command/movement", Int8, command_cb)
    #---TAP TO MOVE---
    TtMC_get = rospy.Subscriber("/barracuda_kinematic/TtMC/target", Pose2D, TtMC_cb)
    #---ENCODER---counter1
    encoder_get = rospy.Subscriber("/barracuda_kinematic/odometry/eksternal/xy", Pose2D, encoder_cb)
    #---ENCODER---counter2
    encoder_get = rospy.Subscriber("/barracuda_kinematic/odometry/eksternal/xy", Pose2D, encoder_cb)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        mainRunV2()
        rate.sleep()