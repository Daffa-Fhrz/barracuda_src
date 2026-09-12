#!/usr/bin/env python3
"""
barracuda_roscom_node.py

Node ROS 'barracuda_roscom':
- Subscribe ke topic-topic yang dipublish oleh basestation (basestation-web,
  lihat public/app.js) yaitu: command, target pose, reset, coordinate, dan
  teammate pose.
- Setiap data yang diterima langsung di-publish ulang ke topic internal
  robot masing-masing (1 data -> 1 topic sendiri).

Referensi topic dari basestation-web (public/app.js):

PUBLISHER basestation -> robot:
    /base_station/command/data   std_msgs/Int8        (kode perintah wasit/kontrol)
    /robot/pose                  geometry_msgs/Pose2D (target pose absolute_pose)
    /robot/reset                 std_msgs/Empty       (belum dipakai di UI, disiapkan)
    /robot/coordinate            geometry_msgs/Pose2D (belum dipakai di UI, disiapkan)
    /robot/teammate_pose         geometry_msgs/Pose2D (relay pose robot pasangan)

Catatan:
- /robot/reset dan /robot/coordinate di sisi basestation memang belum ada
  .publish() yang memanggilnya (slot belum dipakai), tapi subscriber di sini
  tetap disiapkan supaya begitu basestation mulai publish, robot langsung
  bisa menerimanya tanpa perlu ubah kode lagi.
- Nama topic /robot/teammate_pose masih bisa diganti (TEAMMATE_POSE_TOPIC di
  basestation), sesuaikan kalau berubah.
"""

import rospy
from std_msgs.msg import Int8, Empty
from geometry_msgs.msg import Pose2D


class BarracudaRoscom:
    def __init__(self):
        rospy.init_node("barracuda_roscom", anonymous=False)

        # ---------------- Publisher internal robot ----------------
        # Tiap data dari basestation punya topic publish sendiri di sisi robot.
        self.pub_command = rospy.Publisher(
            "/barracuda/command", Int8, queue_size=10
        )
        self.pub_target_pose = rospy.Publisher(
            "/barracuda/target_pose", Pose2D, queue_size=10
        )
        self.pub_reset = rospy.Publisher(
            "/barracuda/reset", Empty, queue_size=10
        )
        self.pub_coordinate = rospy.Publisher(
            "/barracuda/coordinate", Pose2D, queue_size=10
        )
        self.pub_teammate_pose = rospy.Publisher(
            "/barracuda/teammate_pose", Pose2D, queue_size=10
        )

        # ---------------- Subscriber ke basestation ----------------
        rospy.Subscriber(
            "/base_station/command/data", Int8, self.cb_command
        )
        rospy.Subscriber(
            "/robot/pose", Pose2D, self.cb_target_pose
        )
        rospy.Subscriber(
            "/robot/reset", Empty, self.cb_reset
        )
        rospy.Subscriber(
            "/robot/coordinate", Pose2D, self.cb_coordinate
        )
        rospy.Subscriber(
            "/robot/teammate_pose", Pose2D, self.cb_teammate_pose
        )

        rospy.loginfo("[barracuda_roscom] Node siap.")

    # ---------------- Callback dari basestation ----------------
    # Tiap callback langsung publish ulang ke topic internal robot.

    def cb_command(self, msg):
        self.pub_command.publish(msg)

    def cb_target_pose(self, msg):
        self.pub_target_pose.publish(msg)

    def cb_reset(self, msg):
        self.pub_reset.publish(msg)

    def cb_coordinate(self, msg):
        self.pub_coordinate.publish(msg)

    def cb_teammate_pose(self, msg):
        self.pub_teammate_pose.publish(msg)


def main():
    BarracudaRoscom()
    rospy.spin()


if __name__ == "__main__":
    main()
