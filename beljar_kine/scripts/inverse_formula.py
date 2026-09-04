#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class robot_kinematic:
    def __init__(self):
        rospy.init_node('inverse_fx')
        self.cmd_vel_sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.inverse_kinematic)
        self.wheel_speed_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.L = 0.24
        self.R = 0.052
        self.pwm_max = 255
        self.cosin45 = 0.707106781186547524

    def inverse_kinematic(self, twist_msg):
        vx = twist_msg.linear.x 
        vy = twist_msg.linear.y  
        vtheta = twist_msg.angular.z



        matrik_sincos = np.array([
            [self.cosin45, self.cosin45, self.L],
            [-self.cosin45, self.cosin45, self.L],
            [-self.cosin45, -self.cosin45, self.L],
            [self.cosin45, -self.cosin45, self.L]
        ]) / self.R

        vx = np.clip(vx, -self.pwm_max, self.pwm_max)
        vy = np.clip(vy, -self.pwm_max, self.pwm_max)
        vtheta = np.clip(vtheta, -self.pwm_max, self.pwm_max)

        #manipulator (kuwalik cok)
        vy *=-1 

        matriks_input = np.array([vx, vy, vtheta])
        kecepatan_roda = matrik_sincos @ matriks_input
        kecepatan_roda = np.clip(kecepatan_roda, -self.pwm_max, self.pwm_max)

        self.publish_joint_states(kecepatan_roda)

    def publish_joint_states(self, kecepatan_roda):
        speed_msg = JointState()
        speed_msg.name = ['w1', 'w2', 'w3', 'w4']
        speed_msg.velocity = kecepatan_roda.tolist()
        self.wheel_speed_pub.publish(speed_msg)

if __name__ == "__main__":
    try:
        robot_kinematic()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
