#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class InverseKinematicsNode:
    def __init__(self, radius_roda=0.052, radius_robot=0.24, pwm_max=255):

        self.r = radius_roda  # Radius of the wheel in meters
        self.L = radius_robot  # Radius of the robot in meters
        self.pwm_max = pwm_max  # Maximum PWM value

        # ROS Node Initialization
        rospy.init_node('inverse_kinematics_node')

        # Subscriber to velocity commands
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.compute_wheel_speeds)

        # Publisher for wheel speeds
        self.wheel_speed_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def compute_wheel_speeds(self, twist_msg):
        """
        Callback function to compute wheel speeds from Twist message
        :param twist_msg: Twist message containing vx, vy, and vtheta
        """
        # Extract velocity inputs from the Twist message
        vx = twist_msg.linear.x  # Linear velocity along X-axis
        vy = twist_msg.linear.y  # Linear velocity along Y-axis
        vtheta = twist_msg.angular.z  # Angular velocity around Z-axis

        vx *= -1
        vtheta *= -1
        # Kinematic matrix
        matriks_inverse = np.array([
            [ 1,  1,  self.L],  # Roda 1 (depan-kanan)
            [-1,  1,  self.L],   # Roda 2 (belakang-kanan)
            [-1, -1,  self.L],  # Roda 3 (belakang-kiri)
            [ 1, -1,  self.L]  # Roda 4 (depan-kiri)
        ]) / self.r

        # Clip input values to the PWM range
        vx = np.clip(vx, -self.pwm_max, self.pwm_max)
        vy = np.clip(vy, -self.pwm_max, self.pwm_max)
        vtheta = np.clip(vtheta, -self.pwm_max, self.pwm_max)

        # Input matrix
        matriks_input = np.array([vx, vy, vtheta])

        # Compute wheel speeds
        kecepatan_roda = matriks_inverse @ matriks_input

        # Scaling the wheel speeds to match input scale
        input_terbesar = max(abs(vx), abs(vy), abs(vtheta))
        if input_terbesar > 0:  # Avoid division by zero
            kecepatan_roda *= input_terbesar / max(abs(kecepatan_roda))

        # Clip the wheel speeds to the PWM range
        kecepatan_roda = np.clip(kecepatan_roda, -self.pwm_max, self.pwm_max)

        self.publish_joint_states(kecepatan_roda)

    def publish_joint_states(self, kecepatan_roda):
        # Publish the wheel speeds
        speed_msg = JointState()
        speed_msg.name = ['w1', 'w2', 'w3', 'w4']
        speed_msg.velocity = kecepatan_roda.tolist()
        self.wheel_speed_pub.publish(speed_msg)


if __name__ == '__main__':
    try:
        # Initialize the node with default parameters
        InverseKinematicsNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
