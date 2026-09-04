#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class OmniKinematics:
    def __init__(self):
        rospy.init_node('omni_kinematics')
        self.cmd_vel_sub = rospy.Subscriber('/turtle1/cmd_vel', Twist, self.forward_kinematics)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def forward_kinematics(self, twist_msg):
        vx, vy, wz = twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z
        # Wheel radius (r), distance to wheel (L)
        r, L = 0.052, 0.240
        
        # The wheel speed matrix based on the new wheel configuration
        # [w1, w2, w3, w4] -> [front right, back right, back left, front left]
        # Each row corresponds to the influence of vx, vy, wz on each wheel's speed
        wheel_speeds = np.array([
            [1, -1, -L],  # w1 (front right)
            [1,  1,  L],  # w2 (back right)
            [1,  1, -L],  # w3 (back left)
            [1, -1,  L]   # w4 (front left)
        ]) @ np.array([vx, vy, wz]) / r
        
        # Publish the joint states
        self.publish_joint_states(wheel_speeds)

    def publish_joint_states(self, wheel_speeds):
        joint_state = JointState()
        joint_state.name = ['w1', 'w2', 'w3', 'w4']
        joint_state.velocity = wheel_speeds.tolist()
        self.joint_state_pub.publish(joint_state)

if __name__ == "__main__":
    try:
        OmniKinematics()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
