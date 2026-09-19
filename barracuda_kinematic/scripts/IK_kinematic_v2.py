#!/usr/bin/env python3
import rospy
import numpy as np
import math
from geometry_msgs.msg import Pose2D, Point
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
from std_msgs.msg import Bool, Int16, Int8, Int32, Float32, Byte
from geometry_msgs.msg import Pose2D
# from barracuda_kinematic.PID_controller import PIDController # type: ignore
from enum import Enum   

# FIX (jitter pas bola di belakang): filter sudut biasa (rata-rata angka
# derajat mentah) SALAH di sekitar batas wrap +-180, misal rata-rata dari
# 179 dan -179 keitung jadi 0 -- padahal harusnya ketemu di 180/-180.
# Makanya filternya dilakuin di ruang vektor (sin/cos) baru di-atan2 balik
# jadi derajat, biar smoothing-nya "muter" bukan "lurus" pas ngelewatin
# batas itu. alpha kecil = lebih halus/lambat, alpha gede = lebih responsif
# tapi kurang nahan noise.
def circular_lowpass(prev_angle_deg, new_angle_deg, alpha=0.3):
    prev_rad = np.deg2rad(prev_angle_deg)
    new_rad = np.deg2rad(new_angle_deg)
    x = (1 - alpha) * np.cos(prev_rad) + alpha * np.cos(new_rad)
    y = (1 - alpha) * np.sin(prev_rad) + alpha * np.sin(new_rad)
    return float(np.rad2deg(np.arctan2(y, x)))


# FIX (transisi omni<->front / cold-start PID kerasa nyentak): biar
# perubahan wheel speed antar-loop ga langsung lompat jauh (misal dari -90
# ke 90 dalam 1 loop 10Hz), dibatasin naik/turunnya per loop. Reset PID di
# atas udah ngilangin salah-arahnya, tapi transisi fisiknya bisa tetep
# kerasa "nyentak" krn beda karakter kontrol (muter di tempat vs maju
# nyamperin) -- slew limit ini nahan lonjakan mekanisnya.
# Naikin MAX_WHEEL_DELTA_PER_LOOP kalau mau responnya lebih gesit,
# turunin kalau masih kerasa nyentak.
MAX_WHEEL_DELTA_PER_LOOP = 40

def slew_limit_wheel(current_val, target_val, max_delta=MAX_WHEEL_DELTA_PER_LOOP):
    delta = target_val - current_val
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return int(current_val + delta)


class robotState(Enum):
    STOP = 0
    GOTO_ABSPOSE = 1
    GOTO_BALL = 2
    DRIBLE_BALL = 3
    WAIT_BALL = 4
    WAIT_FRIEND = 5
    PRESSING = 6
    NOBALLS = 7
    # [STRAT] state tambahan buat integrasi strategi (nilai 0-7 TIDAK diubah)
    TTMC = 8        # tap-to-move manual (target dari /barracuda_kinematic/TtMC/target)
    SHOOT = 9       # tahan bola, hadap ke targetPose.theta, lalu tendang penuh
    PASS = 10       # sama seperti SHOOT tapi tendang pelan (passing)
    FACE_BALL = 11  # putar di tempat menghadap bola (tanpa maju)


class shootMode(Enum):
    """Mode yang dikirim ke arduino/kicker.
    0 = state biasa, ga nendang
    1 = passing (nendang pelan)
    2 = shooting (nendang penuh)
    """
    NORMAL = 0
    PASSING = 1
    SHOOTING = 2


targetPosex = 0
targetPosey =0
targetPosetheta = 0
currentPosex = 0
currentPosey = 0
currentPosetheta = 0

ball_point_omni = ballInfo()
ball_pos_omni = ballTravel()
ballStatus_omni = False

# FIX: nyimpen hasil filter sudut omni antar-loop (dipakai circular_lowpass
# di ballTravel_cb_omni). Direset ke 0 tiap kali bola omni ilang, biar pas
# ketemu bola lagi ga kebawa "ingatan" filter dari sudut lama yang udah gak
# relevan.
filtered_omni_degree = 0.0
# FIX (salah arah sesaat pas bola baru kedeteksi): nandain apakah
# filtered_omni_degree ini udah pernah diisi bacaan asli atau belum. Tanpa
# ini, begitu bola baru kedeteksi, filter bakal nge-blend dari nilai reset
# (0.0) ke bacaan asli secara PELAN (krn alpha kecil) -> beberapa loop
# pertama nilainya masih "nyasar" deket 0 walau bola sebenernya udah kebaca
# di sudut lain, jadi PID sempet nyuruh muter ke arah yang salah dulu.
omni_filter_initialized = False

ball_point_front = ballInfo()
ball_pos_front = ballTravel()
ballStatus_front = False
# FIX: filter sudut yg sama kayak omni, dipasang juga di front cam.
# Sebelumnya cuma omni yang difilter, front dibiarin mentah -- makanya
# masih ada gejala plus-minus pas transisi ke/dari front cam.
filtered_front_degree = 0.0
front_filter_initialized = False

targetPose = Pose2D()
poseNow = currentPose()
proximityState = ballCatch()         #PROXIMITY SENSOR   

command = robotState.STOP.value
currentShootMode = shootMode.NORMAL.value

# Flag "kunci" fase capture bola di goToBallState_yTheta. Sengaja global &
# one-way-latch: begitu true, dia TIDAK dicek balik ke false gara-gara
# distance_now yang kebaca naik lagi (efek distorsi kamera pas deket bola).
# Cuma direset manual di titik-titik yang aman (proxy kena / bola ilang total).
is_capturing_ball = False

# FIX (kick pas cold-start / handover): nandain PID sudut mana yang lagi
# "megang kendali" loop sebelumnya. Dipakai di goToBallState_yTheta buat
# ngedeteksi kapan sebuah PID baru AKTIF LAGI (abis nganggur, entah krn
# bola sempat ilang atau krn gantian kendali dari kamera lain) -> saat itu
# PID-nya di-reset() dulu biar ga kebawa state basi.
front_angle_pid_active = False
omni_angle_pid_active = False

headingBMM = 0
headingBNO = 0.0

# encoder eksternal (dari arduino/encEksternal/counter1 & counter2, Int32)
encoderCounter1 = 0
encoderCounter2 = 0

speed = speedRobot()    
speedPenggiring = penggiring()

#PUBLISHER SETUP    
wheellSpeed_pub = rospy.Publisher('/barra_kinematic/wheel/speed', speedRobot, queue_size=10)   
penggiring_pub = rospy.Publisher('/barra_kinematic/penggiring/speed', penggiring, queue_size=3)

# TODO: topic shooting mode masih placeholder, GANTI kalau nama topic final
# sudah disepakati sama tim firmware/arduino (sesuaikan namespace-nya juga).
SHOOT_MODE_TOPIC = '/arduino/shoot/mode'
shootMode_pub = rospy.Publisher(SHOOT_MODE_TOPIC, Byte, queue_size=10)

# =====================================================================
# [STRAT] BAGIAN BARU: jembatan ke node strategi (unlimited_2026.py)
# =====================================================================
# Umpan balik ke strategi. Strategi TIDAK lagi nunggu /action_executor/*.
status_pub      = rospy.Publisher('/barracuda_kinematic/status', Int8, queue_size=10)        # 1=busy, 0=done
ballReached_pub = rospy.Publisher('/barracuda_kinematic/ball_reached', Bool, queue_size=10)  # bola di dribbler (kedua proximity)
ballGlobal_pub  = rospy.Publisher('/barracuda_kinematic/ball/global', Point, queue_size=10)  # posisi bola KOORDINAT GLOBAL (cm)

# --- Konvensi sudut (SATU frame buat semua): theta odometry ---
# Dari matriks rotasi di RunTtMC: theta=0 -> depan robot = +y dunia, dan
# theta positif = putar berlawanan jarum jam (CCW), diukur dalam DERAJAT.
# Semua sudut yang dikirim strategi (targetPose.theta) pakai frame ini.
#
# Sudut relatif bola dari kamera: 90 = tepat di depan (sesuai setpoint PID).
# ARAH: derajat kamera NAIK ke arah KANAN robot (searah jarum jam). Buktinya ada di kode ini sendiri:
#   - goToBallState: vx = forward_speed * sin(angle_now - setpoint)  -> pembacaan > setpoint = bola di KANAN (+x lokal)
#   - PID_angle_* memakai (90 - angle_now) dengan putaran yang sama dengan PID_target_theta (theta naik = CCW)
# Jadi bearing bola (CCW dari +y) = theta - (derajat - 90)  -> BALL_REL_SIGN = -1.
# (Kalau di tes bola di kiri robot malah muncul di kanan pada /barracuda_kinematic/ball/global, ubah jadi +1.)
BALL_REL_SIGN = -1.0
# Offset kamera -> pusat robot (CM, sama dengan satuan jarak kamera), kalau jarak dari YOLO diukur dari kamera.
FRONT_CAM_OFFSET_CM = 0.0
OMNI_CAM_OFFSET_CM = 0.0

# --- Toleransi selesai (dipakai buat kirim status "done" ke strategi) ---
ABS_MAX_SPEED = 55            # sama seperti RunTtMC sebelumnya
# SATUAN: semua koordinat (odometry, targetPose, TtMC, titik referee box) dipakai APA ADANYA, tanpa skala
# -> sama dengan basestation (mm). Toleransi sampai di bawah juga dalam satuan yang sama.
ABS_POS_TOL = 80.0            # mm (8 cm): dianggap SAMPAI kalau selisih posisi <= ini (dan heading <= ABS_THETA_TOL). Param: _abs_pos_tol
                              # KECIL = presisi, tapi PID jarak tanpa suku integral + deadband motor bisa bikin robot berhenti sedikit
                              # sebelum titik. Kalau di robot log sering 'Dianggap sampai (zona hampir-sampai)', BESARKAN nilai ini.
# Satu-satunya konversi: jarak bola dari kamera/YOLO (cm) -> satuan odometry (mm), cuma buat posisi bola global.
# Kalau jarak dari YOLO kalian sudah dalam mm, ubah jadi 1.0.
CAMERA_DIST_TO_ODOM = 10.0
# --- Dribbler (penggiring) ---
DRIBBLER_APPROACH_SPEED = 50   # pelan pas ngejar / nunggu operan (nilai dari komentar lama di GOTO_BALL)
DRIBBLER_RELEASE_ON_KICK = True  # True: dribbler dimatiin pas nendang biar bola ga ketahan roller
ABS_THETA_TOL = 5.0           # derajat. Param: _abs_theta_tol
ABS_STABLE_LOOPS = 3          # harus stabil sekian loop (10Hz) baru dianggap sampai
FACE_TOLERANCE_DEG = 8.0
FACE_STABLE_LOOPS = 3
FRONT_ANGLE_SETPOINT = 110    # sama seperti setpoint front di goToBallState_yTheta
SHOOT_SETTLE_SEC = 1.0        # setelah heading lurus, tahan segini lama sebelum tendang
KICK_FINISH_SEC = 0.5         # setelah kick terkirim, tunggu segini lalu state dianggap selesai

aim_angle = 0.0               # heading tujuan pas nahan/nembak bola (derajat, frame odometry)
action_done = True            # dibaca strategi lewat /barracuda_kinematic/status
resume_cmd = None             # kalau bola lepas pas SHOOT/PASS, lanjutin command ini abis bola ketangkap lagi
aligned_since = None
kick_time = 0.0
abs_stable_count = 0
face_stable_count = 0

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0

def currentThetaDeg():
    # odometry theta itu RADIAN (lihat RunTtMC), strategi kerja dalam derajat
    return math.degrees(currentPosetheta)

class PIDController:
    def __init__(self, Kp, Ki, Kd, MaxValue, MinValue, T=0.02, MaxValue_Integer=2, MinValue_integer=-2,
                 SamplingTime=0.01, limit_EN=True, is_angle=False):
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
        # FIX: flag baru. Kalau True, error (setPoint - measurement) akan
        # di-wrap ke rentang -180..180 derajat, jadi robot selalu muter ke
        # arah rotasi terpendek (paling efisien) alih-alih kadang muter
        # muter jauh cuma karena selisih mentahnya kebaca gede akibat
        # lewat batas 0/360 derajat. HANYA dipakai buat PID yang ngontrol
        # sudut/heading, JANGAN dipakai buat PID posisi/jarak linear (x, y).
        self.is_angle = is_angle

    def PID_Calc(self, setPoint, measurement):
        error = setPoint - measurement

        if self.is_angle:
            # FIX: normalisasi error sudut ke rentang -180..180 derajat
            # supaya arah putar yang dipilih selalu yang paling pendek/efisien.
            error = (error + 180) % 360 - 180

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

    def reset(self, measurement=None):
        """FIX: reset semua state internal (integrator, prevError,
        differentiator, prevMeasurement). WAJIB dipanggil begitu controller
        ini baru AKTIF LAGI setelah sempat nganggur -- misal bola sempat
        ilang terus kedeteksi lagi, atau kendali baru "gantian" dari
        kamera/controller lain. Tanpa ini, PID_Calc pertama abis nganggur
        bakal ngitung differentiator/integrator dari measurement BASI
        (sebelum nganggur), nyebabin lonjakan output aneh sesaat (nyundul
        ke satu arah dulu baru balik ke arah yang bener).

        `measurement` (opsional) diisi bacaan sudut/posisi SAAT INI, biar
        prevMeasurement langsung sinkron dan differentiator loop pertama
        keitung 0 (bukan lonjakan gede krn selisih measurement lama-baru).
        """
        self.integrator = 0
        self.prevError = 0
        self.differentiator = 0
        self.prevMeasurement = measurement if measurement is not None else 0
        self.outVal = 0


class IKMove:
    def __init__(self):
        self.r = 5.2  # Radius dalam cm
        self.L = 24  # Jarak pusat Robot - Roda cm
        # self.pwm_max = 130
        self.xy_max = 1870
        self.theta_max = 55
        self.cossin45 = 0.70710678118654752
        #   ---OMNI CAMERA--- (UNIVERSAL THETA)
        # FIX: is_angle=True -> pilih arah putar terpendek (efisien), bukan
        # selalu satu arah kayak sebelumnya.
        self.PID_angle_omni = PIDController(
            Kp=0.75, Ki=0.000000009, Kd=0.175,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True, is_angle=True
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
        # FIX: is_angle=True juga, sama alasannya kayak PID_angle_omni.
        self.PID_angle_front = PIDController(
            Kp=0.065, Ki=0.000000000043, Kd=0.0057,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True, is_angle=True
        )
        self.PID_distance = PIDController(
            Kp=4, Ki=0.0, Kd=0.01,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
            limit_EN=True
        )
        #   ---BMM- --
        # FIX: is_angle=True, PID ini juga ngontrol heading/sudut.
        self.PID_BMM = PIDController(
            Kp=1.5, Ki=0.0, Kd=0.012,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True, is_angle=True
        )
        #   ---BNO---
        # FIX: is_angle=True, dipakai buat align heading pas mau nendang
        # (lihat blok MEKANISME SHOOTING di dribleState), harus ambil jalur
        # putar terpendek juga.
        self.PID_BNO = PIDController(
            Kp=0.75, Ki=0.000000009, Kd=0.175,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True, is_angle=True
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
            Kp=6, Ki=0.0001, Kd=0.46,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
            limit_EN=True
        )
        self.PID_target_y = PIDController(  
            Kp=6, Ki=0.00041, Kd=0.46,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
            limit_EN=True
        )
        # FIX: is_angle=True karena ini PID buat theta (heading) target.
        self.PID_target_theta = PIDController(          
            Kp=0.45, Ki=0.0, Kd=0.015,
            MaxValue=self.theta_max, MinValue=-1 * self.theta_max,
            limit_EN=True, is_angle=True
        )
        self.PID_distance_x = PIDController(
            Kp=4, Ki=0.0, Kd=0.01,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
            limit_EN=True
        )
        self.PID_distance_y = PIDController(
            Kp=4, Ki=0.0, Kd=0.01,
            MaxValue=self.xy_max, MinValue=-1 * self.xy_max,
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
# FIX (ball "sering ilang" pas robot gerak cepat): akar masalahnya motion
# blur di kamera, tapi di sisi software ini bisa dikurangin dampaknya --
# sebelumnya SEKALI YOLO miss 1 frame doang, ballStatus langsung False dan
# trigger reset PID/transisi state yang berat (liat semua FIX di atas).
# Padahal miss 1 frame gara-gara blur sesaat itu beda sama bola BENERAN
# ilang (keluar FOV / ketutup total). Debounce ini nunggu beberapa frame
# miss BERTURUT-TURUT dulu baru declare ballStatus jadi False; kalau cuma
# blip sesaat, status tetep True & ball_pos_* kepake nilai terakhir yang
# valid (robot ga langsung "kebingungan").
#
# Naikin nilainya kalau kamera/YOLO sering ngeblip pas kondisi normal (tapi
# jangan kegedean, karena makin gede nilainya, makin lambat juga robot
# "sadar" kalau bola BENERAN ilang). Turunin kalau reaksi ke kehilangan
# bola beneran kerasa kelamaan.
BALL_LOST_DEBOUNCE_FRAMES_FRONT = 3
BALL_LOST_DEBOUNCE_FRAMES_OMNI = 3

front_miss_counter = 0
omni_miss_counter = 0

def ballStatus_cb_front(data):
    global ballStatus_front, front_miss_counter
    if data.data:
        front_miss_counter = 0
        ballStatus_front = True
    else:
        front_miss_counter += 1
        if front_miss_counter >= BALL_LOST_DEBOUNCE_FRAMES_FRONT:
            ballStatus_front = False
        # else: dianggep masih ke-detect sementara (nunggu mastiin beneran
        # ilang atau cuma blip/blur sesaat), ballStatus_front TETEP True
def ballStatus_cb_omni(data):
    global ballStatus_omni, omni_miss_counter
    if data.data:
        omni_miss_counter = 0
        ballStatus_omni = True
    else:
        omni_miss_counter += 1
        if omni_miss_counter >= BALL_LOST_DEBOUNCE_FRAMES_OMNI:
            ballStatus_omni = False
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
    global ball_pos_front, filtered_front_degree, front_filter_initialized
    if ballStatus_front:
        if not front_filter_initialized:
            # FIX: sample pertama pas bola baru kedeteksi -> langsung
            # pakai bacaan asli (jangan blend dari 0.0 lama), sama alasan
            # kayak yang omni.
            filtered_front_degree = data.degree
            front_filter_initialized = True
        else:
            filtered_front_degree = circular_lowpass(filtered_front_degree, data.degree, alpha=0.3)
        ball_pos_front = data
        ball_pos_front.degree = filtered_front_degree
    else:
        ball_pos_front.distance = 0
        ball_pos_front.degree = 0
        filtered_front_degree = 0.0
        front_filter_initialized = False
def ballTravel_cb_omni(data):
    global ball_pos_omni, filtered_omni_degree, omni_filter_initialized
    if ballStatus_omni:
        if not omni_filter_initialized:
            # FIX: sample pertama pas bola baru kedeteksi -> langsung pakai
            # bacaan asli, JANGAN di-blend dari filtered_omni_degree lama
            # (yang notabene 0.0 bekas reset), biar ga ada jeda "salah arah"
            # di awal.
            filtered_omni_degree = data.degree
            omni_filter_initialized = True
        else:
            # sample berikutnya -> baru difilter halus kayak biasa
            filtered_omni_degree = circular_lowpass(filtered_omni_degree, data.degree, alpha=0.3)
        ball_pos_omni = data
        ball_pos_omni.degree = filtered_omni_degree
    else:
        ball_pos_omni.distance = 0
        ball_pos_omni.degree = 0
        filtered_omni_degree = 0.0  # reset biar ga kebawa "ingatan" filter lama
        omni_filter_initialized = False  # reset juga, siap buat deteksi berikutnya
def strategy_cb(data):
    global targetPose
    targetPose.x = data.x
    targetPose.y = data.y
    targetPose.theta = data.theta
def current_pose_cb(data):
    global currentPosex, currentPosey, currentPosetheta
    currentPosex = data.x
    currentPosey = data.y
    currentPosetheta = data.theta 
def proximity_cb(data):
    # BUG lama: pakai "global ballCatch" (nama class message-nya, bukan
    # instance globalnya), jadi nilai proximityState nggak pernah keupdate.
    # Sekarang dibetulin supaya nulis ke instance global proximityState.
    global proximityState
    proximityState.ballCatch1 = data.ballCatch1
    proximityState.ballCatch2 = data.ballCatch2
def command_cb(data):
    # [STRAT] tiap command BARU: reset semua state kerja + PID biar mulai bersih,
    # tandai "belum selesai", dan langsung kabari strategi (handshake).
    global command, action_done, aim_angle, resume_cmd, aligned_since
    global shoot_already_sent, dribble_start_time, is_capturing_ball
    global front_angle_pid_active, omni_angle_pid_active
    global abs_stable_count, face_stable_count
    new_cmd = data.data
    command = new_cmd
    action_done = (new_cmd == robotState.STOP.value)
    resume_cmd = None
    aligned_since = None
    shoot_already_sent = False
    dribble_start_time = None
    is_capturing_ball = False
    front_angle_pid_active = False
    omni_angle_pid_active = False
    abs_stable_count = 0
    face_stable_count = 0
    th = currentThetaDeg()
    if new_cmd in (robotState.GOTO_ABSPOSE.value, robotState.TTMC.value):
        iK_move.PID_distance_x.reset(measurement=currentPosex)
        iK_move.PID_distance_y.reset(measurement=currentPosey)
        iK_move.PID_target_theta.reset(measurement=th)
    elif new_cmd in (robotState.DRIBLE_BALL.value, robotState.SHOOT.value, robotState.PASS.value):
        aim_angle = targetPose.theta          # sudut hadap ditentukan STRATEGI
        iK_move.PID_BNO.reset(measurement=th)
    status_pub.publish(Int8(data=0 if action_done else 1))
def TtMC_cb(data):
    global targetPosex, targetPosey, targetPosetheta
    targetPosex = data.x
    targetPosey = data.y
    targetPosetheta = data.theta
def encoderCounter1_cb(data):
    global encoderCounter1
    encoderCounter1 = data.data
def encoderCounter2_cb(data):
    global encoderCounter2
    encoderCounter2 = data.data
#---FUNCTION ROBOT STATE---
iK_move = IKMove()  

def penggiringSpeed(w1, w2):
    global speedPenggiring
    speedPenggiring.Drible1 = w1
    speedPenggiring.Drible2 = w2
    penggiring_pub.publish(speedPenggiring)

def sendShootMode(mode):
    """Publish mode tendangan (0=normal/ga nendang, 1=passing, 2=shooting)
    ke topic SHOOT_MODE_TOPIC. Dipanggil tiap loop biar arduino selalu
    tau kondisi terkini (bukan cuma sekali pas mau nendang)."""
    global currentShootMode
    currentShootMode = mode
    shootMode_pub.publish(Byte(data=mode))

def stopState():
    global speed, is_capturing_ball, front_angle_pid_active, omni_angle_pid_active
    speed.w1 = 0
    speed.w2 = 0
    speed.w3 = 0
    speed.w4 = 0
    wheellSpeed_pub.publish(speed)
    penggiringSpeed(0, 0)   # dribbler mati saat STOP
    sendShootMode(shootMode.NORMAL.value)  # safety: pastikan ga ada mode nendang yang nyangkut
    is_capturing_ball = False  # safety: reset kunci capture kalau dipaksa STOP di tengah proses
    # FIX: reset juga flag PID angle, biar begitu balik lagi ke GOTO_BALL
    # abis dipaksa STOP, PID_angle_front/omni mulai dari state fresh
    # (bukan history sebelum di-STOP).
    front_angle_pid_active = False
    omni_angle_pid_active = False
    
def absPoseState():
    # [STRAT] GOTO_ABSPOSE: pakai target dari strategi (/barracuda_strategy/strategy/targetPose)
    # dan controller RunTtMC yang sudah teruji. Theta target dalam DERAJAT.
    global action_done, abs_stable_count
    pos_err, th_err = driveToPose(targetPose.x, targetPose.y, targetPose.theta, ABS_MAX_SPEED)
    # lagi bawa bola (mis. corner: ambil bola lalu pindah posisi) -> dribbler tetap nahan
    if proximityState.ballCatch1 and proximityState.ballCatch2:
        penggiringSpeed(DRIBBLER_SPEED, DRIBBLER_SPEED)
    else:
        penggiringSpeed(0, 0)
    if pos_err <= ABS_POS_TOL and abs(th_err) <= ABS_THETA_TOL:
        abs_stable_count += 1
    else:
        abs_stable_count = 0
    if abs_stable_count >= ABS_STABLE_LOOPS:
        action_done = True   # robot tetap nahan posisi (PID jalan terus) sampai ada command baru

# --- Konstanta buat fase "capture" (dorong maju buat ambil bola) ---
# Setpoint PID_distance (30) itu bikin robot BERHENTI pas jarak 30, bukan
# nyuruh dia terus maju ngambil bola. Jadi begitu udah cukup deket, kita
# lepas kontrol PID jarak dan ganti dorong konstan sampai proxy kena bola.
# TANDA fase mendekat. Kode asli: forward_speed = PID_Calc(30, jarak) * 5 = 20*(30 - jarak) -> NEGATIF untuk bola
# yang lebih jauh dari 30 cm, padahal fase dorong di bawah (CAPTURE_FORWARD_VY = +500) POSITIF untuk maju, dan
# ABSPOSE/TtMC juga maju = vy positif. Satu variabel (vy) tidak bisa "maju" dengan dua tanda berlawanan.
#   -1.0 (default) : bola jauh -> vy POSITIF (maju), konsisten dengan fase dorong & ABSPOSE
#   +1.0           : perilaku kode asli (kalau di robot ternyata bola dikejar dengan tanda asli, kembalikan ke ini)
CHASE_APPROACH_SIGN = -1.0
CAPTURE_DISTANCE_THRESHOLD = 35   # cm, di bawah ini dianggap "udah deket, mulai push"
CAPTURE_FORWARD_VY = 500          # kecepatan dorong konstan pas capture, sesuaikan/tuning

def goToBallState_yTheta():
    global ballStatus_front, ball_pos_front, ballStatus_omni, ball_pos_omni, command, speed, proximityState, is_capturing_ball
    global front_angle_pid_active, omni_angle_pid_active
    if ballStatus_front:
        distance_now = ball_pos_front.distance 
        angle_now = ball_pos_front.degree

        # FIX: PID_angle_front baru "gantian pegang kendali" (sebelumnya
        # omni yang aktif, atau bola abis sempet ilang) -> reset dulu biar
        # ga ada lonjakan output dari state basi (lihat penjelasan di
        # PIDController.reset()).
        if not front_angle_pid_active:
            iK_move.PID_angle_front.reset(measurement=angle_now)
            front_angle_pid_active = True
        omni_angle_pid_active = False  # omni lagi nganggur selama front aktif

        angle = iK_move.PID_angle_front.PID_Calc(110, angle_now)

        # Sekali kepicu (jarak <= threshold), KUNCI ke mode push -- jangan
        # dicek balik ke distance_now tiap loop, soalnya kalau udah deket,
        # bacaan jarak dari kamera bisa distorsi/naik lagi padahal bola
        # sebenernya masih di depan. Kalau dicek ulang tiap loop, dia bisa
        # ping-pong balik ke mode PID gara-gara bacaan itu.
        if not is_capturing_ball and distance_now <= CAPTURE_DISTANCE_THRESHOLD:
            is_capturing_ball = True

        if not is_capturing_ball:
            # masih jauh -> approach normal pakai PID jarak (target berhenti di 30)
            distance = iK_move.PID_distance.PID_Calc(30, distance_now)
            forward_speed = distance * 5 * CHASE_APPROACH_SIGN
        else:
            # udah kekunci mode push -> dorong maju konstan, ga peduli
            # distance_now abis ini kebaca berapa, sampai proxy kena
            forward_speed = CAPTURE_FORWARD_VY

        # FIX (kaku, kesan "muter dulu baru maju"): SEBELUMNYA translasi
        # cuma lurus badan (vx=0, vy=forward_speed) sementara koreksi
        # sudut (vtheta) itungannya kepisah -- robot jadi kesannya nunggu
        # badannya lurus dulu baru "berani" maju penuh. Padahal base robot
        # ini omni/holonomic (liat formula w1..w4 di perumusan() yang ada
        # vx-nya), jadi SEBENERNYA bisa langsung gerak menyerong (strafe)
        # ke arah bola meskipun badan belum lurus, SAMBIL vtheta pelan2
        # ngeluruskan badan di saat bersamaan -- dua-duanya jalan barengan,
        # bukan bergantian.
        #
        # Arah translasi (vx, vy) di bawah dihitung LANGSUNG condong ke
        # arah bola pakai sin/cos dari deviasi sudut (bukan cuma lurus ke
        # depan badan robot), baru vtheta (dari PID_angle_front, `angle`)
        # tetap jalan buat pelan2 luruskan badan ke arah bola.
        #
        # CATATAN PENTING: tanda sin/cos di bawah based on asumsi konvensi
        # "makin gede angle_now dari 90 = bola makin ke satu sisi tertentu"
        # (sama kayak yang dipakai PID_angle_front). Kalau pas ditest robot
        # malah nyerong ke sisi yang salah, tinggal kasih minus di depan
        # deviation_rad (jadi `-deviation_rad`) -- jangan ubah yang lain.
        deviation_deg = angle_now - 90
        deviation_rad = np.deg2rad(deviation_deg)
        vx = forward_speed * np.sin(deviation_rad)
        vy = forward_speed * np.cos(deviation_rad)

        w1, w2, w3, w4 = iK_move.perumusan(vx, vy, angle, 80)
        # dinegasi biar konsisten sama mainRunV2 (yang udah tervalidasi jalan
        # bener di robot) -- perumusan() yang sama tapi kalau ga diminus,
        # arah gerak robotnya kebalik.
        # FIX: slew-limit dari speed.w* yang lagi dipublish sekarang menuju
        # target baru, biar transisi (misal abis reset PID / gantian dari
        # omni) ga nyentak lompat jauh dalam 1 loop.
        speed.w1 = slew_limit_wheel(speed.w1, -w1)
        speed.w2 = slew_limit_wheel(speed.w2, -w2)
        speed.w3 = slew_limit_wheel(speed.w3, -w3)
        speed.w4 = slew_limit_wheel(speed.w4, -w4)
        wheellSpeed_pub.publish(speed)

        # Nyalain dribbler dari sekarang biar bola ketarik pas robot mepet,
        # begitu proxy kedeteksi baru pindah state ke DRIBLE_BALL.
        penggiringSpeed(DRIBBLER_APPROACH_SPEED, DRIBBLER_APPROACH_SPEED)
        if proximityState.ballCatch1 or proximityState.ballCatch2:
            # [STRAT] bola ketangkap -> masuk hold (atau lanjutin SHOOT/PASS yang tertunda)
            enter_hold_state()
            is_capturing_ball = False  # reset, siap buat siklus GOTO_BALL berikutnya
    elif ballStatus_omni:
        penggiringSpeed(0, 0)
        # bola ilang dari kamera depan -> proses capture batal, reset flag-nya
        # biar kalau ketemu bola lagi dari depan, mulai dari approach normal
        is_capturing_ball = False
        front_angle_pid_active = False  # front lagi nganggur selama omni aktif
        angle_now = ((ball_pos_omni.degree + 180) % 360) - 180

        # FIX: PID_angle_omni baru "gantian pegang kendali" (sebelumnya
        # front yang aktif, atau bola abis sempet ilang total) -> reset
        # dulu, ini yang benerin gejala "kesundul positif dulu baru
        # negatif" pas bola baru kedeteksi lagi.
        if not omni_angle_pid_active:
            iK_move.PID_angle_omni.reset(measurement=angle_now)
            omni_angle_pid_active = True

        # FIX: PID_angle_omni sekarang is_angle=True, jadi arah putarnya
        # otomatis ambil jalur terpendek (efisien) sesuai posisi bola,
        # ga akan lagi keukeuh muter ke satu arah aja.
        angle = iK_move.PID_angle_omni.PID_Calc(90, angle_now)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 90)
        # FIX: slew-limit juga di sini, alasan sama kayak cabang front.
        speed.w1 = slew_limit_wheel(speed.w1, -w1)
        speed.w2 = slew_limit_wheel(speed.w2, -w2)
        speed.w3 = slew_limit_wheel(speed.w3, -w3)
        speed.w4 = slew_limit_wheel(speed.w4, -w4)
        wheellSpeed_pub.publish(speed)
    else:
        penggiringSpeed(0, 0)
        # bola ilang total (front & omni sama-sama ga detect) -> kedua PID
        # sudut dianggap nganggur, biar pas salah satu aktif lagi nanti
        # di-reset dulu (lihat 2 cabang di atas).
        is_capturing_ball = False
        front_angle_pid_active = False
        omni_angle_pid_active = False
        speed.w1 = 0
        speed.w2 = 0    
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)


# ---- Konstanta buat state DRIBLE_BALL, sesuaikan pas tuning ----
DRIBBLER_SPEED = 150            # kecepatan motor dribbler pas narik/nahan bola
HEADING_ALIGN_TOLERANCE = 7.0   # derajat, toleransi sebelum dianggap "udah lurus ke target"

# kecepatan gerak sementara pas bola udah captured, cuma buat testing dribble
# dulu (belom ada logic shooting). Sesuaikan/ganti sesuka hati buat testing.
DRIBBLE_HOLD_DURATION = 2.0
dribble_start_time = None 
DRIBBLE_TEST_VY = 100
DRIBBLE_TEST_VTHETA = 0
shoot_already_sent = False
target_angle = 180

def enter_hold_state():
    """[STRAT] Dipanggil begitu bola kena proximity (dari GOTO_BALL / WAIT_BALL).
    Kalau sebelumnya lagi SHOOT/PASS lalu bola sempat lepas (resume_cmd), lanjutin
    command itu dengan aim_angle yang sama. Kalau tidak, cuma NAHAN bola tanpa muter
    (aim = heading sekarang) -> strategi baru yang nentuin mau hadap/tendang ke mana."""
    global command, aim_angle, aligned_since, shoot_already_sent, dribble_start_time, resume_cmd
    if resume_cmd is not None:
        command = resume_cmd
        resume_cmd = None
    else:
        command = robotState.DRIBLE_BALL.value
        aim_angle = currentThetaDeg()
    aligned_since = None
    shoot_already_sent = False
    dribble_start_time = None
    iK_move.PID_BNO.reset(measurement=currentThetaDeg())

def aimState(kick_mode):
    """[STRAT] Ganti dribleState lama. Bola udah dipegang dribbler:
       - putar di tempat sampai heading (odometry) = aim_angle (dari strategi)
       - kick_mode None       -> cuma nahan (DRIBLE_BALL)
       - kick_mode PASSING/SHOOTING -> tahan SHOOT_SETTLE_SEC setelah lurus, lalu tendang 1x
    Heading pakai ODOMETRY (bukan BNO) supaya satu frame sama sudut dari strategi."""
    global command, action_done, shoot_already_sent, aligned_since, kick_time, resume_cmd
    global is_capturing_ball
    now = rospy.get_time()
    ballCaught = proximityState.ballCatch1 and proximityState.ballCatch2

    # --- kick sudah terkirim: tunggu sebentar, lalu selesai & diam ---
    if shoot_already_sent:
        sendShootMode(shootMode.NORMAL.value)
        if DRIBBLER_RELEASE_ON_KICK:
            penggiringSpeed(0, 0)
        else:
            penggiringSpeed(DRIBBLER_SPEED, DRIBBLER_SPEED)
        if now - kick_time >= KICK_FINISH_SEC:
            shoot_already_sent = False
            action_done = True
            command = robotState.STOP.value      # jangan langsung ngejar bola lagi, tunggu strategi
        return

    if not ballCaught:
        # bola lepas sebelum sempat ditendang -> kejar lagi, lalu lanjutin command ini
        sendShootMode(shootMode.NORMAL.value)
        penggiringSpeed(0, 0)
        if command in (robotState.SHOOT.value, robotState.PASS.value, robotState.DRIBLE_BALL.value):
            resume_cmd = command if command != robotState.DRIBLE_BALL.value else None
        command = robotState.GOTO_BALL.value
        aligned_since = None
        is_capturing_ball = False
        return

    penggiringSpeed(DRIBBLER_SPEED, DRIBBLER_SPEED)   # bola ketangkap -> roller nahan bola
    theta_deg = currentThetaDeg()
    heading_error = wrap180(aim_angle - theta_deg)

    if abs(heading_error) < HEADING_ALIGN_TOLERANCE:
        speed.w1 = 0
        speed.w2 = 0
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)
        if aligned_since is None:
            aligned_since = now
        if kick_mode is None:
            sendShootMode(shootMode.NORMAL.value)
            action_done = True
        elif now - aligned_since >= SHOOT_SETTLE_SEC:
            sendShootMode(kick_mode)
            if DRIBBLER_RELEASE_ON_KICK:
                penggiringSpeed(0, 0)
            shoot_already_sent = True
            kick_time = now
        else:
            sendShootMode(shootMode.NORMAL.value)
    else:
        aligned_since = None
        vtheta = iK_move.PID_BNO.PID_Calc(aim_angle, theta_deg)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, vtheta, 90)
        speed.w1 = slew_limit_wheel(speed.w1, -w1)
        speed.w2 = slew_limit_wheel(speed.w2, -w2)
        speed.w3 = slew_limit_wheel(speed.w3, -w3)
        speed.w4 = slew_limit_wheel(speed.w4, -w4)
        wheellSpeed_pub.publish(speed)
        sendShootMode(shootMode.NORMAL.value)

def dribleState():
    aimState(None)

def faceBallStep():
    """[STRAT] Putar di tempat menghadap bola (tanpa translasi).
    Return error sudut (derajat) atau None kalau bola tidak terlihat."""
    global front_angle_pid_active, omni_angle_pid_active
    if ballStatus_front:
        angle_now = ball_pos_front.degree
        if not front_angle_pid_active:
            iK_move.PID_angle_front.reset(measurement=angle_now)
            front_angle_pid_active = True
        omni_angle_pid_active = False
        angle = iK_move.PID_angle_front.PID_Calc(FRONT_ANGLE_SETPOINT, angle_now)
        err = wrap180(angle_now - FRONT_ANGLE_SETPOINT)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 80)
    elif ballStatus_omni:
        angle_now = ((ball_pos_omni.degree + 180) % 360) - 180
        if not omni_angle_pid_active:
            iK_move.PID_angle_omni.reset(measurement=angle_now)
            omni_angle_pid_active = True
        front_angle_pid_active = False
        angle = iK_move.PID_angle_omni.PID_Calc(90, angle_now)
        err = wrap180(angle_now - 90)
        w1, w2, w3, w4 = iK_move.perumusan(0, 0, angle, 90)
    else:
        front_angle_pid_active = False
        omni_angle_pid_active = False
        speed.w1 = 0
        speed.w2 = 0
        speed.w3 = 0
        speed.w4 = 0
        wheellSpeed_pub.publish(speed)
        return None
    speed.w1 = slew_limit_wheel(speed.w1, -w1)
    speed.w2 = slew_limit_wheel(speed.w2, -w2)
    speed.w3 = slew_limit_wheel(speed.w3, -w3)
    speed.w4 = slew_limit_wheel(speed.w4, -w4)
    wheellSpeed_pub.publish(speed)
    return err

def waitBallState():
    """WAIT_BALL: hadap bola dan nunggu operan. Selesai (done) begitu bola kena proximity."""
    penggiringSpeed(DRIBBLER_APPROACH_SPEED, DRIBBLER_APPROACH_SPEED)   # roller pelan biar operan gampang ketangkap
    if proximityState.ballCatch1 or proximityState.ballCatch2:
        enter_hold_state()
        return
    faceBallStep()

def faceBallState():
    """FACE_BALL: hadap bola; done kalau error sudut < toleransi beberapa loop berturut-turut."""
    global action_done, face_stable_count
    penggiringSpeed(0, 0)
    err = faceBallStep()
    if err is not None and abs(err) < FACE_TOLERANCE_DEG:
        face_stable_count += 1
    else:
        face_stable_count = 0
    if face_stable_count >= FACE_STABLE_LOOPS:
        action_done = True

def waitFriendState():
    pass   # belum dipakai

def pressingState():
    pass   # belum dipakai

def noBallsState():
    pass   # belum dipakai (search bola belum ada)


#MAIN PROGRAM
def runProgram():
    global command
    if command == robotState.STOP.value:
        rospy.loginfo_throttle(2.0, "STOP STATE")
        stopState()
    elif command == robotState.GOTO_ABSPOSE.value:
        rospy.loginfo_throttle(1.0, "GOTO_ABSPOSE")
        absPoseState()
    elif command == robotState.GOTO_BALL.value:
        rospy.loginfo_throttle(1.0, "GOTO_BALL")
        goToBallState_yTheta()
    elif command == robotState.DRIBLE_BALL.value:
        rospy.loginfo_throttle(1.0, "DRIBLE_BALL (hold)")
        aimState(None)
    elif command == robotState.SHOOT.value:
        rospy.loginfo_throttle(1.0, "SHOOT")
        aimState(shootMode.SHOOTING.value)
    elif command == robotState.PASS.value:
        rospy.loginfo_throttle(1.0, "PASS")
        aimState(shootMode.PASSING.value)
    elif command == robotState.WAIT_BALL.value:
        rospy.loginfo_throttle(1.0, "WAIT_BALL")
        waitBallState()
    elif command == robotState.FACE_BALL.value:
        rospy.loginfo_throttle(1.0, "FACE_BALL")
        faceBallState()
    elif command == robotState.TTMC.value:
        RunTtMC()
    else:
        # command belum diimplementasi (WAIT_FRIEND/PRESSING/NOBALLS) -> aman: diam
        stopState()

def publishBallGlobal():
    """[STRAT] Ubah bacaan relatif kamera (jarak, sudut) jadi koordinat GLOBAL bola
    pakai odometry. Dipublish cuma saat bola terlihat. Front diprioritaskan."""
    if ballStatus_front and ball_pos_front.distance > 0:
        dist = ball_pos_front.distance + FRONT_CAM_OFFSET_CM
        deg = ball_pos_front.degree
    elif ballStatus_omni and ball_pos_omni.distance > 0:
        dist = ball_pos_omni.distance + OMNI_CAM_OFFSET_CM
        deg = ball_pos_omni.degree
    else:
        return
    dist = dist * CAMERA_DIST_TO_ODOM   # jarak kamera (cm) -> satuan odometry (mm)
    bearing = math.radians(currentThetaDeg() + BALL_REL_SIGN * wrap180(deg - 90.0))
    # depan robot pada theta=0 adalah +y, theta+ = CCW  ->  arah = (-sin, cos)
    bx = currentPosex - dist * math.sin(bearing)
    by = currentPosey + dist * math.cos(bearing)
    ballGlobal_pub.publish(Point(x=bx, y=by, z=0.0))

def publishFeedback():
    status_pub.publish(Int8(data=0 if action_done else 1))
    ballReached_pub.publish(Bool(data=bool(proximityState.ballCatch1 and proximityState.ballCatch2)))
    publishBallGlobal()

def mainRun():
    if ballStatus_omni:
        angle_now = ball_pos_omni.degree
        # FIX: pakai PID_angle_omni yang sekarang is_angle=True -> arah
        # putar otomatis ambil yang paling pendek/efisien.
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
        # FIX: sama, PID_angle_omni is_angle=True -> pilih arah rotasi
        # terpendek berdasarkan posisi bola relatif ke robot.
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

ALIGN_TOLERANCE_RAD = math.radians(5)  # toleransi align heading sebelum boleh translasi

def driveToPose(tx, ty, ttheta_deg, max_speed=55):
    """Isi lama RunTtMC, sekarang bisa dipanggil dengan target apa saja.
    x,y = cm (frame odometry), ttheta_deg = DERAJAT. Return (jarak_error, error_theta_deg)."""
    global speed

    # --- Translasi (tidak diubah) ---
    x_global = iK_move.PID_distance_x.PID_Calc(tx, currentPosex)
    y_global = iK_move.PID_distance_y.PID_Calc(ty, currentPosey)

    theta_rad = currentPosetheta
    theta_deg = math.degrees(theta_rad)

    x_local =  x_global * math.cos(theta_rad) + y_global * math.sin(theta_rad)
    y_local = -x_global * math.sin(theta_rad) + y_global * math.cos(theta_rad)

    # --- Rotasi (tidak diubah) ---
    vtheta = iK_move.PID_target_theta.PID_Calc(ttheta_deg, theta_deg)

    rospy.loginfo_throttle(0.5,
        f"target=({tx:.1f},{ty:.1f},{ttheta_deg:.1f}) | "
        f"current=({currentPosex:.1f},{currentPosey:.1f},{theta_deg:.1f}) | "
        f"local=({x_local:.1f},{y_local:.1f}) | vtheta={vtheta}"
    )

    w1, w2, w3, w4 = iK_move.perumusan(x_local, y_local, vtheta, max_speed)
    speed.w1 = -w1
    speed.w2 = -w2
    speed.w3 = -w3
    speed.w4 = -w4
    wheellSpeed_pub.publish(speed)

    pos_err = math.hypot(tx - currentPosex, ty - currentPosey)
    th_err = wrap180(ttheta_deg - theta_deg)
    return pos_err, th_err

def RunTtMC():
    # tap-to-move manual (target dari /barracuda_kinematic/TtMC/target) -> jalannya sama persis kayak dulu
    driveToPose(targetPosex, targetPosey, targetPosetheta, ABS_MAX_SPEED)


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
    headingBNO_get = rospy.Subscriber("/arduino/theta/heading", Float32, bno_heading)
    proximity_get = rospy.Subscriber("/arduino/ball/catch", ballCatch, proximity_cb)   
    # ---STRATEGY---
    strategy_get = rospy.Subscriber("/barracuda_strategy/strategy/targetPose", Pose2D, strategy_cb)
    #---CURRENT POSE---
    current_pose_get = rospy.Subscriber("/robot/kinematic/odometry/pose", Pose2D, current_pose_cb)    
    #---COMMAND---
    command_get = rospy.Subscriber("/barracuda_strategy/command/movement", Int8, command_cb)
    #---TAP TO MOVE---
    TtMC_get = rospy.Subscriber("/barracuda_kinematic/TtMC/target", Pose2D, TtMC_cb)
    #---ENCODER EKSTERNAL---
    encoderCounter1_get = rospy.Subscriber("arduino/encEksternal/counter1", Int32, encoderCounter1_cb)
    encoderCounter2_get = rospy.Subscriber("arduino/encEksternal/counter2", Int32, encoderCounter2_cb)
    rate = rospy.Rate(10) # 10Hz
    # [STRAT] loop utama sekarang jalanin state machine (runProgram) + kirim umpan balik ke strategi.
    # Mau tes tap-to-move kayak kemarin tanpa strategi:  rosrun ... IK_kinematic.py _start_command:=8
    ABS_POS_TOL = float(rospy.get_param("~abs_pos_tol", ABS_POS_TOL))          # mm
    ABS_THETA_TOL = float(rospy.get_param("~abs_theta_tol", ABS_THETA_TOL))    # derajat
    command = rospy.get_param("~start_command", robotState.STOP.value)
    action_done = (command == robotState.STOP.value)
    while not rospy.is_shutdown():
        runProgram()
        publishFeedback()
        rate.sleep()