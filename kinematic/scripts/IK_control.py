#!/usr/bin/env python3
# ==============================================
# Bagian Import Module
# ==============================================

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from camera.msg import ballStraightPoint, ballPoint

# ==============================================
# Bagian Inisialisasi
# ==============================================


# ==============================================
# Bagian Class
# ==============================================
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
        self.limit_EN = True

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

        if self.limit_EN == 1:
            if self.outVal > self.limMax:
                self.outVal = self.limMax
            elif self.outVal < self.limMin:
                self.outVal = self.limMin
        else:
            pass

        self.prevError = error
        self.prevMeasurement = measurement

        return int(self.outVal)  # Konversi ke integer


class ik_move:
    def __init__(self):
        # --NODE INITIALIZATION--
        rospy.init_node('kinematicIK_node', anonymous=False)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.mainRun)
        self.r = 5.2  # Radius dalam cm
        self.L = 24  # Jarak pusat Robot - Roda cm
        self.pwm_max = 255
        self.xy_max = 1870
        self.theta_max = 55
        self.cossin45 = 0.70710678118654752
        self.omniBall_data = None
        self.fishBall_data = None
        # --PID SETTING
        self.PID_angle = PIDController(
            Kp=1, Ki=0.001, Kd=0.002,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True
        )
        # --SUBSCRIBER TOPIC--
        rospy.Subscriber('/straightPos_Deg/omniCam', ballStraightPoint, self.omniBall_callBack)
        # --PUBLISHER TOPIC--
        self.wheelSpeed_pub = rospy.Publisher('/kinematic/wheel_speed', Int32MultiArray, queue_size=10)

    # ==========================================
    # Fungsi Callback
    # ==========================================
    def omniBall_callBack(self, data):
        self.omniBall_data = data

    def fishBall_callBack(self, data):
        self.fishBall_data = data

    # ==========================================
    # Fungsi Utama
    # ==========================================
    def perumusan(self, vx, vy, vtheta):
        pos_array = [vx, vy]
        long_pos = abs(pos_array[0])

        for pos in pos_array:
            if abs(pos) > long_pos:
                long_pos = abs(pos)
        if long_pos > self.xy_max:
            vx = int(vx / long_pos * self.xy_max)
            vy = int((vy / long_pos * self.xy_max) * (-1))

        vtheta = int(np.clip(vtheta, -self.theta_max, self.theta_max))

        w1 = int((vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r)
        w2 = int((-vx * self.cossin45 + vy * self.cossin45 + vtheta * self.L) / self.r)
        w3 = int((-vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r)
        w4 = int((vx * self.cossin45 - vy * self.cossin45 + vtheta * self.L) / self.r)

        speed_arrray = [w1, w2, w3, w4]

        hgh_speed = abs(speed_arrray[0])

        for speed in speed_arrray:
            if abs(speed) > hgh_speed:
                hgh_speed = abs(speed)

        if hgh_speed > self.pwm_max:
            w1 = int(w1 / hgh_speed * self.pwm_max)
            w2 = int(w2 / hgh_speed * self.pwm_max)
            w3 = int(w3 / hgh_speed * self.pwm_max)
            w4 = int(w4 / hgh_speed * self.pwm_max)

        return w1, w2, w3, w4

    # MAIN RUN
    def mainRun(self, event):
        try:
            if self.omniBall_data is not None:
                omniCam_dist = int(self.omniBall_data.distance)  # Konversi ke integer
                omniCam_angle = int(self.omniBall_data.degree)  # Konversi ke integer
                if omniCam_angle is not None and omniCam_dist is not None:
                    pidOut_angle = self.PID_angle.PID_Calc(90, omniCam_angle)
                    if pidOut_angle is not None:
                        w1, w2, w3, w4 = self.perumusan(0, 0, pidOut_angle)
                        speed_msg = Int32MultiArray()
                        speed_msg.data = [w1, w2, w3, w4]
                        self.wheelSpeed_pub.publish(speed_msg)
                        rospy.logerr(pidOut_angle)
                    else:
                        rospy.logerr("Invalid PID output. Skipping calculation.")
                elif pidOut_angle is None or omniCam_angle is None and omniCam_dist is None:
                    pidOut_angle = 0
                    w1, w2, w3, w4 = 0
                    speed_msg = Int32MultiArray()
                    speed_msg.data = [w1, w2, w3, w4]
                    self.wheelSpeed_pub.publish(speed_msg)
            else:
                rospy.logwarn("omniBall_data is None. Waiting for data...")
        except Exception as e:
            rospy.logerr(f"Error dalam mainRun: {e}")


# ==============================================
# Program Berjalan (if __name__ == '__main__')
# ==============================================
if __name__ == '__main__':
    try:
        ik_node = ik_move()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass