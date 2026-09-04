import rospy
from geometry_msgs.msg import Pose2D

class base_station:
    def __init__(self):
        self.pose_global = Pose2D()

    def int_com_cb(self, data):
        self.pose_global = data

if __name__ == '__main__':
    rospy.init_node('odometry_com_base_station')
    base = base_station()
    rospy.Subscriber('/odometry/internal/xy', Pose2D, base.int_com_cb)
    rospy.Subscriber('/robot/pose', Pose2D, base.int_com_cb)
