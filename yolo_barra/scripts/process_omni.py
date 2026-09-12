#!/usr/bin/env python3

import rospy
import math
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

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

from yolo_barra.shapegui import (
    line,
    text,
    crossPlus
)


class algo:

    def __init__(self):

        # FIX: sebelumnya sama-sama pakai nama "YoloProcessBarra" kayak
        # process_front.py -- kalau dijalanin bareng tanpa remap nama di
        # launch file, ROS bakal nendang salah satu (nama node HARUS unik
        # per proses). Sekarang dikasih suffix biar beda.
        rospy.init_node("YoloProcessBarra_omni")

        # =========================
        # SUBSCRIBER
        # =========================

        self.yolo = rospy.Subscriber(
            "omni/yolo_result",
            YoloResult,
            self.dataCallback
        )

        self.image_get = rospy.Subscriber(
            "omni/yolo_image",
            Image,
            self.mergeCallback
        )

        # =========================
        # PUBLISHER
        # =========================

        # FIX: topic ballInfo/ballTravel/ballStatus di bawah ini dikasih
        # suffix "_yolo" -- ini jadi topic MENTAH (bukan final), yang
        # dengerin sekarang bukan kinematic lagi, tapi node vision_fusion.py
        # yang gabungin hasil YOLO ini sama hasil HSV, baru fusion-nya yang
        # publish ke topic final (/Barracuda_Yolo/omni/ballInfo dst) yang
        # didengerin kinematic. Lihat vision_fusion.py.
        self.ballInfoPub = rospy.Publisher(
            "/Barracuda_Yolo/omni/ballInfo_yolo",
            ballInfo,
            queue_size=10
        )

        self.ballTravelPub = rospy.Publisher(
            "/Barracuda_Yolo/omni/ballTravel_yolo",
            ballTravel,
            queue_size=10
        )

        self.robotPub = rospy.Publisher(
            "/Barracuda_Yolo/omni/robotInfo",
            robotInfoArray,
            queue_size=10
        )

        self.goalPub = rospy.Publisher(
            "/Barracuda_Yolo/omni/goalInfo",
            goalInfo,
            queue_size=10
        )

        self.dummyPub = rospy.Publisher(
            "/Barracuda_Yolo/omni/dummyInfo",
            dummyInfoArray,
            queue_size=10
        )

        self.mergepub = rospy.Publisher(
            "/Barracuda_Yolo/omni/MergeResult",
            Image,
            queue_size=10
        )

        # FIX: sebelumnya SAMA PERSIS sama topic yg dipakai hsvOmni.py --
        # kalau dua-duanya jalan bareng, tabrakan publisher & kinematic
        # nerima status ngaco dari 2 sumber independen. Dikasih suffix
        # "_yolo" biar jadi topic mentah, digabung di vision_fusion.py.
        self.ballStatus_pub = rospy.Publisher("/barracuda_vision/camera/omni/ballStatus_yolo", Bool, queue_size=1)

        # =========================
        # CV BRIDGE
        # =========================

        self.bridge = CvBridge()

        # =========================
        # CAMERA PARAMETER
        # =========================

        self.CF_x = 320
        self.CF_y = 240

        # =========================
        # DISTANCE CALIBRATION
        # =========================
        # Satuan output bebas, tetapi harus konsisten dengan data kalibrasi.
        # Contoh: kalau data ukur pakai cm, maka hasil distance juga cm.
        # Model inverse_linear cocok untuk objek yang ukuran bbox-nya makin kecil
        # saat objek makin jauh: distance = m * (1 / pixel_size) + c

        self.distance_model = {
            # Nilai awal ini mengikuti rumus lama:
            # distance = (focal_length * real_size) / pixel_size
            # Ganti nilai m dan c setelah kalibrasi lapangan.
            "robot": {
                "model": "inverse_linear",
                "m": 5000 * 0.40,
                "c": 0.0
            },
            "goal": {
                "model": "inverse_linear",
                "m": 5000 * 1.20,
                "c": 0.0
            },
            "dummy": {
                "model": "inverse_linear",
                "m": 5000 * 0.50,
                "c": 0.0
            }
        }

        # =========================
        # DATA STORAGE
        # =========================

        self.ball_data = None
        self.ballTravel = None

        self.robot_data = []
        self.goal_data = None
        self.dummy_data = []

    # ==================================================
    # CALCULATE DEGREE
    # ==================================================

    def calculateDegree(self, x, y):

        dx = x - self.CF_x
        dy = self.CF_y - y

        angle_rad = math.atan2(dy, dx)

        angle_deg = math.degrees(angle_rad)

        if angle_deg < 0:
            angle_deg += 360

        return int(angle_deg)

    # ==================================================
    # CALCULATE DISTANCE
    # ==================================================

    def omniBallDistance(self, r):
        distance = (
            0.00339097505 * (r ** 2)
            + 0.168195431 * r
            - 6.49405699
        )

        return max(0, distance)
    
    def objectDistance(self, object_name, pixel_size):
        """
        Menghitung jarak objek berdasarkan hasil kalibrasi/regresi.

        object_name : "robot", "goal", atau "dummy"
        pixel_size  : ukuran bbox yang dipakai sebagai acuan.
                      robot/dummy biasanya height, goal biasanya width.
        """

        if pixel_size <= 0:
            return 0

        cfg = self.distance_model.get(object_name)

        if cfg is None:
            return 0

        model = cfg["model"]

        if model == "linear":
            # distance = m * pixel_size + c
            distance = (
                cfg["m"] * pixel_size
                + cfg["c"]
            )

        elif model == "inverse_linear":
            # distance = m * (1 / pixel_size) + c
            distance = (
                cfg["m"] * (1.0 / pixel_size)
                + cfg["c"]
            )

        elif model == "quadratic":
            # distance = a * pixel_size^2 + b * pixel_size + c
            distance = (
                cfg["a"] * (pixel_size ** 2)
                + cfg["b"] * pixel_size
                + cfg["c"]
            )

        else:
            distance = 0

        return max(0, distance)

    # ==================================================
    # YOLO CALLBACK
    # ==================================================

    def dataCallback(self, msg):

        # RESET DATA
        self.ball_data = None
        self.robot_data = []
        self.goal_data = None
        self.dummy_data = []

        # EMPTY DETECTION
        if not msg.detections.detections:
            return

        # ARRAY MESSAGE
        robot_array = robotInfoArray()
        dummy_array = dummyInfoArray()

        # LOOP DETECTION
        for detection in msg.detections.detections:

            if not detection.results:
                continue

            class_id = detection.results[0].id
            score = detection.results[0].score

            x = int(detection.bbox.center.x)
            y = int(detection.bbox.center.y)

            width = int(detection.bbox.size_x)
            height = int(detection.bbox.size_y)

            # ==================================================
            # BALL
            # ==================================================

            if class_id == 0:

                self.ball_data = ballInfo()

                self.ball_data.x = x
                self.ball_data.y = y
                self.ball_data.radius = int(height / 2)
                self.ball_data.score = score

                self.ballInfoPub.publish(self.ball_data)

                # TRAVEL
                dlx = x - self.CF_x
                dly = y - self.CF_y
                r = math.sqrt(dlx**2 + dly**2)
                self.ballTravel = ballTravel()

                self.ballTravel.degree = self.calculateDegree(x, y)

                self.ballTravel.distance = self.omniBallDistance(r)

                self.ballTravelPub.publish(self.ballTravel)

            # ==================================================
            # ROBOT
            # ==================================================

            elif class_id == 1:

                robot = robotInfo()

                robot.x = x
                robot.y = y

                robot.width = width
                robot.height = height

                robot.score = score

                robot.distance = self.objectDistance(
                    "robot",
                    height
                )

                robot.degree = self.calculateDegree(
                    x,
                    y
                )

                robot_array.robots.append(robot)

                self.robot_data.append(robot)

            # ==================================================
            # GOAL
            # ==================================================

            elif class_id == 2:

                goal = goalInfo()

                goal.x = x
                goal.y = y

                goal.width = width
                goal.height = height

                goal.score = score

                goal.distance = self.objectDistance(
                    "goal",
                    width
                )

                goal.degree = self.calculateDegree(
                    x,
                    y
                )

                self.goal_data = goal

                self.goalPub.publish(goal)

            # ==================================================
            # DUMMY
            # ==================================================

            elif class_id == 3:

                dummy = dummyInfo()

                dummy.x = x
                dummy.y = y

                dummy.width = width
                dummy.height = height

                dummy.score = score

                dummy.distance = self.objectDistance(
                    "dummy",
                    height
                )

                dummy.degree = self.calculateDegree(
                    x,
                    y
                )

                dummy_array.dummies.append(dummy)

                self.dummy_data.append(dummy)

        # PUBLISH ARRAY
        self.robotPub.publish(robot_array)
        self.dummyPub.publish(dummy_array)

    # ==================================================
    # IMAGE CALLBACK
    # ==================================================

    def mergeCallback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(
            msg,
            "bgr8"
        )

        if cv_image is None:
            rospy.logwarn("Frame tidak diterima")
            return

        frame_ui = cv_image

        # CROSSHAIR
        crossPlus(frame_ui)

        # ==================================================
        # BALL DEBUG
        # ==================================================

        if self.ball_data is not None:

            
            line(
                frame_ui,
                self.CF_x,
                self.CF_y,
                self.ball_data.x,
                self.ball_data.y,
                "BIRU",
                2
            )

            text(
                frame_ui,
                f"BALL X:{self.ball_data.x} Y:{self.ball_data.y}, rad:{self.ball_data.radius}, distance:{self.ballTravel.distance}, degree:{self.ballTravel.degree}",
                20,
                30,
                "SIMPLEX",
                0.5,
                "HIJAU",
                1
            )

            self.ballStatus_pub.publish(1)


        else:

            text(
                frame_ui,
                "BALL : NONE",
                20,
                30,
                "SIMPLEX",
                0.5,
                "MERAH",
                1
            )

            self.ballStatus_pub.publish(0)


        # ==================================================
        # ROBOT DEBUG
        # ==================================================

        robot_y = 60

        if len(self.robot_data) > 0:

            for i, robot in enumerate(self.robot_data):

                text(
                    frame_ui,
                    f"ROBOT-{i} X:{robot.x} Y:{robot.y}, distance:{robot.distance:.2f}, degree:{robot.degree}",
                    20,
                    robot_y,
                    "SIMPLEX",
                    0.5,
                    "HIJAU",
                    1
                )

                robot_y += 25

        else:

            text(
                frame_ui,
                "ROBOT : NONE",
                20,
                robot_y,
                "SIMPLEX",
                0.5,
                "MERAH",
                1
            )

        # ==================================================
        # GOAL DEBUG
        # ==================================================

        if self.goal_data is not None:

            text(
                frame_ui,
                f"GOAL X:{self.goal_data.x} Y:{self.goal_data.y}, distance:{self.goal_data.distance:.2f}, degree:{self.goal_data.degree}",
                20,
                robot_y + 20,
                "SIMPLEX",
                0.5,
                "HIJAU",
                1
            )

        else:

            text(
                frame_ui,
                "GOAL : NONE",
                20,
                robot_y + 20,
                "SIMPLEX",
                0.5,
                "MERAH",
                1
            )

        # ==================================================
        # DUMMY DEBUG
        # ==================================================

        dummy_y = robot_y + 50

        if len(self.dummy_data) > 0:

            for i, dummy in enumerate(self.dummy_data):

                text(
                    frame_ui,
                    f"DUMMY-{i} X:{dummy.x} Y:{dummy.y}, distance:{dummy.distance:.2f}, degree:{dummy.degree}",
                    20,
                    dummy_y,
                    "SIMPLEX",
                    0.5,
                    "HIJAU",
                    1
                )

                dummy_y += 25

        else:

            text(
                frame_ui,
                "DUMMY : NONE",
                20,
                dummy_y,
                "SIMPLEX",
                0.5,
                "MERAH",
                1
            )

        # ==================================================
        # PUBLISH IMAGE
        # ==================================================

        self.mergepub.publish(
            self.bridge.cv2_to_imgmsg(
                frame_ui,
                "bgr8"
            )
        )


# ==================================================
# MAIN
# ==================================================

if __name__ == "__main__":

    try:

        algo()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass