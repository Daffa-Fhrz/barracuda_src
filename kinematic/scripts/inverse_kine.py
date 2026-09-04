#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from barracuda_roscom.msg import speedRobot

class OmniIKNode:
    def __init__(self):
        rospy.init_node('omni_inverse_kinematic')

        # Parameter robot (sama dengan kode kamu)
        self.r = 5.2
        self.L = 24.0
        self.c45 = 0.70710678118
        self.pwm_max = 255

        rospy.Subscriber('/cmd_vel', Twist, self.cmdVelCallback)
        self.pub_wheel = rospy.Publisher('/barracuda_kinematic/wheel/speed', speedRobot, queue_size=10)

        rospy.loginfo("Omni IK Node using speedRobot msg started")

    def cmdVelCallback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        vtheta = msg.angular.z

        w1, w2, w3, w4 = self.inverse_kinematic(vx, vy, vtheta)

        wheel_msg = speedRobot()
        wheel_msg.w1 = w1
        wheel_msg.w2 = w2
        wheel_msg.w3 = w3
        wheel_msg.w4 = w4

        self.pub_wheel.publish(wheel_msg)

    def inverse_kinematic(self, vx, vy, vtheta):
        # Rumus sama persis dengan perumusan kamu
        w1 = ( vx*self.c45 + vy*self.c45 + self.L*vtheta ) / self.r
        w2 = (-vx*self.c45 + vy*self.c45 + self.L*vtheta ) / self.r
        w3 = (-vx*self.c45 - vy*self.c45 + self.L*vtheta ) / self.r
        w4 = ( vx*self.c45 - vy*self.c45 + self.L*vtheta ) / self.r

        max_speed = max(abs(w1), abs(w2), abs(w3), abs(w4))

        if max_speed > self.pwm_max:
            scale = self.pwm_max / max_speed
            w1 *= scale
            w2 *= scale
            w3 *= scale
            w4 *= scale

        return int(w1), int(w2), int(w3), int(w4)


if __name__ == '__main__':
    try:
        OmniIKNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
