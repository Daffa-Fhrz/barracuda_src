#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Strategi KRSBI-B 2026 (Caca & Cici) -- versi yang jalan di atas IK_kinematic_v2.py

Satu file buat dua robot:
    rosrun <pkg> unlimited_2026.py _robot_name:=caca
    rosrun <pkg> unlimited_2026.py _robot_name:=cici

Yang DIPERTAHANKAN dari unlimited_2026_Caca/Cici.py:
    - role (Attacker/Defender), GameState, dan logika tiap state
    - cara menghadap gawang pas shooting (LEFT_GOAL / RIGHT_GOAL) & arah passing
    - state machine referee box (Enable) + tabel Pose

Yang DIGANTI: semua jalur ke robot sekarang lewat topic & perhitungan IK_kinematic
    (lihat tabel topic di bawah). Node ini TIDAK lagi butuh /action_executor/*.

    command      -> /barracuda_strategy/command/movement       (Int8, enum robotState kinematic)
    target pose  -> /barracuda_strategy/strategy/targetPose    (Pose2D, cm & DERAJAT)
    posisi robot <- /robot/kinematic/odometry/pose             (Pose2D, theta RADIAN)
                    (dikonversi ke frame GLOBAL referee box lewat kalibrasi di reset(),
                     jadi TIDAK butuh topic set-pose di node odometry)
    selesai/busy <- /barracuda_kinematic/status                (Int8: 1 busy, 0 done)
    bola dipegang<- /barracuda_kinematic/ball_reached          (Bool)
    bola global  <- /barracuda_kinematic/ball/global           (Point, satuan odometry)

ALUR REFEREE BOX (semua set piece menunggu juri menekan Play / Enable.OnPlay):
    klik set piece -> robot ke titik referee box, lalu STOP (diam total) -> juri tekan Play -> baru bergerak
    Keluar dari Play (juri kirim Stop / set piece baru) -> roda langsung STOP.
    Enable.ResetOdometry (17) -> kalibrasi ulang frame lapangan ke titik Initial (odometry tidak disentuh).
    Home (kita) : KickOffHome -> PASSING DULU: kicker (param _kickoff_kicker) ambil bola & oper ke teman;
                  penerima diam menghadap bola, baru ambil kalau bola sudah <= 1 m dari dia.
                  Lainnya (GoalKick/FreeKick/ThrowIn/DropBall) -> langsung strategi biasa (Play).
                  CornerHome / PenaltyHome -> skrip ambil bola & tendang setelah Play.
    Away (lawan): setelah Play, tunggu bola bergeser 1 m ATAU 7 detik (mana duluan), baru main.

FRAME KOORDINAT: sama dengan basestation -> titik nol di POJOK KIRI BAWAH lapangan (kuadran 1),
x ke kanan, y ke depan (arah serang), satuan tabel = mm, heading/yaw dalam derajat.
Lapangan default 8000 x 12000 mm (_field_width_mm / _field_length_mm). Gawang lawan di y = panjang.

SATUAN: semua koordinat dipakai APA ADANYA, TANPA faktor skala -- titik referee box, TtMC, targetPose,
dan odometry memakai satuan yang sama (mm, sama dengan basestation). Jarak/toleransi di file ini juga mm.
Kalibrasi frame: robot dianggap BERDIRI di Pose 'Initial' saat node strategi start
(atau saat /robot/reset). Set _calibrate_on_start:=false kalau odometry sudah global.
"""
import rospy
from time import sleep
from math import atan2, degrees, radians, sqrt, sin, cos
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Int8, Bool, Empty
from enum import Enum

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# KONFIGURASI
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# Konvensi sudut = frame ODOMETRY kinematic: 0 deg = depan robot menghadap +y,
# positif = putar berlawanan jarum jam (CCW).  Rumus lama getAngle = atan2(dx, dy)
# itu searah jarum jam, jadi dikali -1.
# >>> TES: suruh robot hadap titik di kanan (+x). Kalau muternya kebalik, ganti jadi +1.0
ANGLE_SIGN = -1.0

# Kalau True: bola bebas -> robot yang paling dekat NGEJAR bola (Attacking_Attacker).
# Kalau False: perilaku persis kode lama (robot terdekat malah ke attackerPosition
# nunggu operan, dan robot satunya cuma nge-block).
CHASE_FREE_BALL = True

# --- Aturan set piece ---
# SEMUA set piece menunggu juri menekan Play (OnPlay) sebelum robot mulai main/menendang.
# True : setelah juri klik set piece, robot jalan ke titik referee box lalu DIAM menahan posisi
#        sampai Play.  False: robot tidak bergerak sama sekali (STOP) sampai Play.
SET_PIECE_MOVE_TO_POSE = True
# Menuju titik referee box: robot TETAP di state ABSPOSE sampai "sampai" (lihat toleransi bertingkat di bawah).
# Stop / set piece lain dari juri langsung membatalkan. RESTART_POSITION_TIMEOUT (detik) = batas pengaman;
# None = tunggu tanpa batas waktu.
RESTART_POSITION_TIMEOUT = 40.0
# TOLERANSI BERTINGKAT (supaya tidak macet mengejar titik yang "terlalu spesifik"):
#   1) kinematic melapor SAMPAI  : selisih <= ABS_POS_TOL (80 mm) dan heading <= ABS_THETA_TOL (5 deg)  [di kinematic]
#   2) PENYELAMAT anti-macet (zona "hampir sampai"): kalau robot bertahan di dalam ARRIVE_SOFT_POS / ARRIVE_SOFT_THETA
#      selama ARRIVE_SOFT_SETTLE detik tanpa kinematic melapor sampai (mis. deadband motor) -> dianggap SAMPAI (di-STOP).
#   3) PLAY MENUNGGU SAMPAI: juri menekan Play sebelum robot sampai -> strategi baru mulai setelah robot sampai (1 atau 2).
#      Pengaman: kalau tetap belum sampai PLAY_START_GRACE detik setelah Play -> strategi mulai dari posisi sekarang
#      (supaya robot tidak diam selama pertandingan berjalan). PLAY_START_IN_SOFT_ZONE = True -> boleh mulai
#      segera begitu masuk zona hampir-sampai (tanpa menunggu ARRIVE_SOFT_SETTLE).
#   4) RESTART_POSITION_TIMEOUT: belum sampai juga (mis. macet/terhalang) -> berhenti mencoba (STOP), tunggu Play.
ARRIVE_SOFT_POS = 150          # mm
ARRIVE_SOFT_THETA = 8.0        # deg
ARRIVE_SOFT_SETTLE = 3.0       # detik
PLAY_START_GRACE = 10.0        # detik
PLAY_START_IN_SOFT_ZONE = False
# Setelah robot SAMPAI di titik referee box dan menunggu Play:
#   'stop' : perintah STOP (roda & dribbler mati) -> robot diam total sampai juri tekan Play
#   'hold' : tetap GOTO_ABSPOSE (PID terus menahan posisi; motor tetap aktif mengoreksi)
WAIT_PLAY_HOLD = 'stop'

# Kick-off (dan restart lain di PASS_FIRST_RESTARTS): PASSING DULU.
# Robot "kicker" ambil bola lalu oper ke teman. Robot "penerima" diam menghadap bola dan
# baru mengambil bola kalau bola sudah <= RECEIVE_TRIGGER (1 m) dari dirinya.
# Siapa kicker: param  _kickoff_kicker:=caca|cici  (HARUS sama di kedua robot).
KICKOFF_KICKER = 'caca'
RECEIVE_TRIGGER = 1000       # mm (1 m): jarak bola ke penerima -> penerima mulai ngambil
KICKOFF_MIN_BALL_TRAVEL = 300 # mm: bola harus sudah bergeser segini dari posisi awal (tanda sudah dioper)
KICKOFF_RECEIVE_TIMEOUT = 10.0  # detik: penerima nunggu bola; lewat ini -> main normal
KICKOFF_SEE_BALL_TIMEOUT = 3.0  # detik: kicker nunggu bola terlihat kameranya; lewat ini -> main normal
KICKER_POST_PASS_WAIT = 6.0     # detik: kicker nunggu teman ambil bola; lewat ini -> main normal
# Kicker SESUDAH mengoper:
#   'support' : geser ke titik dukung (sisi lapangan berlawanan dari penerima, agak maju, hadap teman)
#               lalu siaga -> jadi opsi operan balik, dan tidak menghalangi / mengejar bola yang baru dioper
#   'stay'    : diam di tempat menghadap bola
KICKER_AFTER_PASS = 'support'
SUPPORT_FORWARD = 2000        # mm: titik dukung sejauh ini ke depan dari garis tengah panjang
SUPPORT_LATERAL = 1000        # mm: titik dukung sejauh ini ke samping dari garis tengah lebar, sisi berlawanan penerima
SUPPORT_MIN_SEPARATION = 1200 # mm: robot tidak saling deteksi -> titik dukung dijaga minimal segini dari teman
# Operan melenceng: kalau bola sudah berhenti (geser < BALL_STOPPED dalam BALL_STOPPED_SEC) setelah
# dioper tapi masih > 1 m dari penerima, penerima tetap jalan mengambilnya.
BALL_STOPPED = 100             # mm
BALL_STOPPED_SEC = 0.6

# Set piece LAWAN: nunggu sampai bola bergeser BALL_MOVED atau AWAY_WAIT_TIMEOUT detik.
AWAY_WAIT_TIMEOUT = 7.0

# ---- Lapangan & frame koordinat ----
# FRAME GLOBAL (sama dengan basestation/referee box): titik nol di POJOK KIRI BAWAH lapangan,
# x ke kanan (lebar), y ke depan menuju gawang lawan (panjang) -> semua koordinat KUADRAN 1.
# Titik = [x, y, yaw] dengan x,y dalam mm (basestation), yaw dalam derajat.
# Lapangan KRSBI Beroda 8 m x 12 m (sesuai angka di tabel: x=4000 & y~6000 = garis tengah).
# Ubah lewat param  _field_width_mm  _field_length_mm  kalau lapangan kalian beda.
FIELD_WIDTH_MM = 8000      # sumbu x
FIELD_LENGTH_MM = 12000    # sumbu y (arah serang: gawang lawan di y = FIELD_LENGTH)

# Yaw di tabel -> heading frame strategi/odometry (0 = hadap +y, positif = CCW):
#     heading = TABLE_YAW_SIGN * yaw + TABLE_YAW_OFFSET_DEG
# Default: yaw basestation SAMA dengan konvensi kinematic. Kalau yaw basestation 0 deg = hadap +x
# (standar matematika, CCW) -> _yaw_offset_deg:=-90.  Kalau searah jarum jam -> _yaw_sign:=-1.
TABLE_YAW_SIGN = 1.0
TABLE_YAW_OFFSET_DEG = 0.0

# Semua jarak di bawah dalam MM = satuan yang sama dengan titik referee box / TtMC / odometry.
# Yang relatif terhadap garis tengah lapangan ditulis sebagai OFFSET.
SAFE_RADIUS = 3000               # mm (3 m) dari gawang sendiri
BLOCK_DIST = 1000                # mm (1 m)
INTERCEPT_DIST = 1200            # mm
DEFENSE_AREA_OFFSET = -1000      # "area aman" = y < garis tengah - 1 m (setengah lapangan sendiri)
ATTACK_READY_OFFSET = 1000       # teman dianggap "siap diserahi bola" kalau y > garis tengah + 1 m
DEFAULT_DEFENSE_OFFSET = -3000   # posisi jaga kalau bola tidak diketahui: 3 m di belakang garis tengah
GOAL_POST_OFFSET = 800           # target tembak: +-80 cm dari tengah gawang (LEFT_GOAL / RIGHT_GOAL)
FIELD_MARGIN = 300               # titik yang DIHITUNG robot dijaga minimal segini dari pagar

BALL_MEMORY_SEC = 2.0       # bola dianggap "masih diketahui" segini lama setelah terakhir terlihat
TEAMMATE_TIMEOUT = 2.0      # teman dianggap hilang kalau tidak ada pose segini lama
BALL_MOVED = 1000           # mm: set piece lawan -> bola dianggap sudah ditendang kalau geser segini (1 m)


def setField(field_w_mm=None, field_l_mm=None):
    """Hitung titik-titik lapangan (frame pojok kiri bawah, mm) dari ukuran lapangan."""
    global FIELD_WIDTH_MM, FIELD_LENGTH_MM, FIELD_W, FIELD_L, CX, CY
    global goal, defense_goal, LEFT_GOAL, RIGHT_GOAL, DEFENSE_AREA, ATTACK_READY_Y
    global DEFAULT_DEFENSE_POS, SUPPORT_Y
    if field_w_mm is not None:
        FIELD_WIDTH_MM = field_w_mm
    if field_l_mm is not None:
        FIELD_LENGTH_MM = field_l_mm
    FIELD_W = float(FIELD_WIDTH_MM)
    FIELD_L = float(FIELD_LENGTH_MM)
    CX = FIELD_W / 2.0                  # garis tengah lebar
    CY = FIELD_L / 2.0                  # garis tengah panjang
    goal = [CX, FIELD_L]                # gawang lawan (kita menyerang ke +y)
    defense_goal = [CX, 0.0]            # gawang sendiri
    LEFT_GOAL = [CX - GOAL_POST_OFFSET, FIELD_L]
    RIGHT_GOAL = [CX + GOAL_POST_OFFSET, FIELD_L]
    DEFENSE_AREA = CY + DEFENSE_AREA_OFFSET
    ATTACK_READY_Y = CY + ATTACK_READY_OFFSET
    DEFAULT_DEFENSE_POS = [CX, CY + DEFAULT_DEFENSE_OFFSET]
    SUPPORT_Y = CY + SUPPORT_FORWARD


def clampToField(x, y):
    """Jaga titik yang dihitung robot tetap di dalam lapangan (kuadran 1) dengan margin dari pagar."""
    x = min(max(x, FIELD_MARGIN), FIELD_W - FIELD_MARGIN)
    y = min(max(y, FIELD_MARGIN), FIELD_L - FIELD_MARGIN)
    return [x, y]


setField()


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

class Enable(Enum):
    Stop = 0
    OnPlay = 1
    StartPosition = 2
    DropBall = 3
    KickOffHome = 4
    KickOffAway = 5
    GoalKickHome = 6
    GoalKickAway = 7
    FreeKickHome = 8
    FreeKickAway = 9
    CornerHome = 10
    CornerAway = 11
    PenaltyHome = 12
    PenaltyAway = 13
    ThrowInHome = 14
    ThrowInAway = 15
    Empty = 16
    # Additional
    ResetOdometry = 17
    KickBall = 18


class Move(Enum):
    """HARUS sama persis dengan robotState di IK_kinematic_v2.py"""
    Stop = 0
    GotoAbsPose = 1
    GotoBall = 2
    DribleBall = 3      # tahan bola + hadap ke targetPose.theta (tanpa tendang)
    WaitBall = 4        # hadap bola, nunggu operan (done saat bola kena)
    Shoot = 9           # hadap targetPose.theta lalu tendang penuh
    Pass = 10           # hadap targetPose.theta lalu tendang pelan
    FaceBall = 11       # hadap bola (done saat sudah lurus)


class Role(Enum):
    Attacker = 0
    Defender = 1


class GameState(Enum):
    Attacking_Attacker = 0
    Attacking_Defender = 1
    Defending_Defender = 2
    Defending_Attacker = 3


# ---- Titik referee box per robot. ISI/KOREKSI DI SINI setelah titik referee box fix. ----
# Format [x, y, theta_deg]; theta dalam frame odometry (0 = hadap +y, CCW +).
# Angka di bawah disalin apa adanya dari unlimited_2026_Caca.py / _Cici.py.
POSES = {
    'caca': {
        'Initial':      [250, -150, 0],
        'DropBall':     [2000, 5000, 0],
        'KickOffHome':  [6000, 3800, 45],
        'KickOffAway':  [2100, 3200, 0],
        'GoalKickHome': [2100, 1460, 0],
        'GoalKickAway': [3000, 3200, 0],
        'FreeKickHome': [2000, 5400, 0],
        'FreeKickAway': [2850, 3300, 22],
        'CornerHome':   [-250, 100, 100],
        'CornerAway':   [2700, 1000, 0],
        'PenaltyHome':  [0, 0, 0],
        'PenaltyAway':  [0, 0, 0],
    },
    'cici': {
        'Initial':      [6000, -150, 0],
        'DropBall':     [2000, 5000, 0],
        'KickOffHome':  [2000, 5900, 270],
        'KickOffAway':  [4000, 4300, 0],
        'GoalKickHome': [-2100, 1460, 0],
        'GoalKickAway': [4000, 4200, 0],
        'FreeKickHome': [-2000, 5400, 0],
        'FreeKickAway': [-2850, 3300, 338],
        'CornerHome':   [-250, 100, 100],
        'CornerAway':   [-2700, 1000, 0],
        'PenaltyHome':  [4000, 5500, 0],
        'PenaltyAway':  [8000, 0, 0],
    },
}


class PoseTable(object):
    """Akses gaya lama: Pose.KickOffHome -> [x, y, heading]; x,y (mm) TANPA skala, yaw -> heading."""
    def __init__(self, table):
        for name, (x, y, th) in table.items():
            heading = (TABLE_YAW_SIGN * th + TABLE_YAW_OFFSET_DEG + 180.0) % 360.0 - 180.0
            setattr(self, name, [x, y, heading])          # x,y dipakai APA ADANYA (mm)


Pose = PoseTable(POSES['caca'])   # diganti di __main__ sesuai ~robot_name
IS_CACA = True
IS_KICKER = True                # robot ini yang menendang kick-off (lihat KICKOFF_KICKER)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# STATE GLOBAL
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

enable = Enable.Stop
role = Role.Attacker
gameState = GameState.Defending_Defender

currentPose = Pose2D()          # frame GLOBAL, theta DERAJAT (dikonversi dari odometry)
odomPose = Pose2D()             # mentah dari odometry, theta DERAJAT
poseReceived = False

# Transformasi odometry -> global:  global = R(frame_phi) * odom + (frame_tx, frame_ty)
frame_phi = 0.0                 # derajat
frame_tx = 0.0
frame_ty = 0.0
ballReached = False             # robot ini memegang bola (kedua proximity)

teammatePose = Pose2D()
teammateStamp = -1e9
teammateBallReached = False

ownBall = None                  # [x, y] global, dari kamera robot ini
ownBallStamp = -1e9
teammateBall = None             # [x, y] global, dari kamera teman (relay /teammate/ball_global)
teammateBallStamp = -1e9

kinBusy = False                 # status terakhir dari kinematic
kinStatusSeen = False

arahPenalty = 0

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# PUBLISHER
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# ---- ke kinematic ----
command_pub = rospy.Publisher('/barracuda_strategy/command/movement', Int8, queue_size=10)
target_pub = rospy.Publisher('/barracuda_strategy/strategy/targetPose', Pose2D, queue_size=10)
# ---- dibagi ke teman (relay eksternal ke /teammate/*) ----
share_pose_pub = rospy.Publisher('/robot/pose', Pose2D, queue_size=10)               # theta DERAJAT
share_ballReached_pub = rospy.Publisher('/robot/ball_reached', Bool, queue_size=10)
share_ball_pub = rospy.Publisher('/robot/ball_global', Point, queue_size=10)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# CALLBACK
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def enHandler(data):
    global enable
    enable = Enable(data.data)
    rospy.loginfo("Local Strategy : " + str(enable))


def odomPoseHandler(data):
    """/robot/kinematic/odometry/pose  (theta RADIAN) -> currentPose (theta DERAJAT)."""
    global currentPose, odomPose, poseReceived
    o = Pose2D()
    o.x = data.x
    o.y = data.y
    o.theta = degrees(data.theta)
    odomPose = o
    p = Pose2D()
    p.x, p.y, p.theta = odomToGlobal(o.x, o.y, o.theta)
    currentPose = p
    poseReceived = True
    share_pose_pub.publish(p)


def kinStatusHandler(data):
    global kinBusy, kinStatusSeen
    kinBusy = (data.data != 0)
    kinStatusSeen = True


def ballReachedHandler(data):
    global ballReached
    ballReached = data.data
    share_ballReached_pub.publish(Bool(data=ballReached))


def ownBallHandler(data):
    global ownBall, ownBallStamp
    gx, gy, _ = odomToGlobal(data.x, data.y, 0.0)     # bola dari kinematic = frame odometry
    ownBall = [gx, gy]
    ownBallStamp = rospy.get_time()
    share_ball_pub.publish(Point(x=gx, y=gy, z=0.0))  # dibagi ke teman dalam frame GLOBAL


def teammateHandler(data):
    global teammatePose, teammateStamp
    teammatePose = data
    teammateStamp = rospy.get_time()


def teammateBallReachedHandler(data):
    global teammateBallReached
    teammateBallReached = data.data


def teammateBallHandler(data):
    global teammateBall, teammateBallStamp
    teammateBall = [data.x, data.y]
    teammateBallStamp = rospy.get_time()


def resetHandler(data):
    global enable
    prevEnable = enable
    enable = Enable.Stop
    sleep(0.3)
    calibrateFrame(Pose.Initial)      # robot diletakkan lagi di Initial -> kalibrasi ulang
    enable = prevEnable
    rospy.loginfo("Local Strategy : Reset")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# UTIL
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def distance(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def getAngle(robotPose, pointTarget):
    """Sudut (derajat, frame odometry) agar robot MENGHADAP pointTarget."""
    return float(ANGLE_SIGN * degrees(atan2(pointTarget[0] - robotPose.x,
                                            pointTarget[1] - robotPose.y)))


def myXY():
    return [currentPose.x, currentPose.y]


def teammateAlive():
    return (rospy.get_time() - teammateStamp) < TEAMMATE_TIMEOUT


def ownBallAge():
    return rospy.get_time() - ownBallStamp


def ballVisible():
    """Kamera robot ini lagi lihat bola (kinematic publish ~10Hz saat terlihat)."""
    return ownBallAge() < 0.5


def getBallPosition():
    """Posisi bola GLOBAL [x, y] atau None kalau tidak ada yang tahu.
    Prioritas: dipegang sendiri > terlihat kamera sendiri > terlihat kamera teman."""
    now = rospy.get_time()
    if ballReached:
        return myXY()
    if ownBall is not None and (now - ownBallStamp) < BALL_MEMORY_SEC:
        return ownBall
    if teammateBall is not None and (now - teammateBallStamp) < BALL_MEMORY_SEC:
        return teammateBall
    return None


def _rot(x, y, deg):
    c, sn = cos(radians(deg)), sin(radians(deg))
    return x * c - y * sn, x * sn + y * c


def odomToGlobal(x, y, theta_deg):
    rx, ry = _rot(x, y, frame_phi)
    return rx + frame_tx, ry + frame_ty, wrap180(theta_deg + frame_phi)


def globalToOdom(x, y, theta_deg):
    rx, ry = _rot(x - frame_tx, y - frame_ty, -frame_phi)
    return rx, ry, wrap180(theta_deg - frame_phi)


def calibrateFrame(initial):
    """Anggap robot SEKARANG berdiri di `initial` ([x, y, theta_deg] frame global).
    Hitung rotasi + translasi odometry -> global, supaya semua koordinat strategi
    (tabel referee box, gawang, posisi teman) nyambung tanpa perlu set-pose ke odometry."""
    global frame_phi, frame_tx, frame_ty
    frame_phi = wrap180(initial[2] - odomPose.theta)
    rx, ry = _rot(odomPose.x, odomPose.y, frame_phi)
    frame_tx = initial[0] - rx
    frame_ty = initial[1] - ry
    p = Pose2D()
    p.x, p.y, p.theta = odomToGlobal(odomPose.x, odomPose.y, odomPose.theta)
    global currentPose
    currentPose = p
    rospy.loginfo("Kalibrasi frame: phi=%.1f deg, t=(%.1f, %.1f)" % (frame_phi, frame_tx, frame_ty))


def reset(calibrate=True):
    rospy.loginfo("Reset")
    t0 = rospy.get_time()
    while not poseReceived and not rospy.is_shutdown():
        if rospy.get_time() - t0 > 5.0:
            rospy.logwarn("Reset: belum ada data odometry (/robot/kinematic/odometry/pose)")
            return
        sleep(0.1)
    if calibrate:
        calibrateFrame(Pose.Initial)
    rospy.loginfo("Reset Done")

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# ROLE & GAME STATE  (logika lama, ditambah penanganan bola/teman tidak diketahui)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def updateRole():
    global role
    isInSafeArea = currentPose.y < DEFENSE_AREA

    # robot dapat bola
    if ballReached:
        role = Role.Defender if isInSafeArea else Role.Attacker
        return

    # teman sendirian di lapangan -> main sebagai penyerang
    if not teammateAlive():
        role = Role.Attacker
        return

    # teman dapat bola
    if teammateBallReached:
        teammateInSafeArea = teammatePose.y < DEFENSE_AREA
        role = Role.Attacker if teammateInSafeArea else Role.Defender
        return

    # bola bebas
    ball = getBallPosition()
    if ball is None:
        return          # tidak ada yang tahu bola -> role tetap
    my_dist = distance(myXY(), ball)
    mate_dist = distance([teammatePose.x, teammatePose.y], ball)
    # Dua robot pakai data yang sama, jadi aturannya harus saling melengkapi:
    # kalau seri, Caca yang jadi Attacker, Cici jadi Defender (bukan dua-duanya sama).
    if IS_CACA:
        role = Role.Attacker if my_dist <= mate_dist else Role.Defender
    else:
        role = Role.Attacker if my_dist < mate_dist else Role.Defender


def updateGameState():
    global gameState
    if ballReached:
        gameState = (GameState.Attacking_Attacker if role == Role.Attacker
                     else GameState.Attacking_Defender)
    else:
        if role == Role.Defender:
            gameState = GameState.Defending_Defender
        else:
            # Attacker tapi belum pegang bola:
            #   teman yang pegang bola -> posisi nunggu operan (logika lama)
            #   bola bebas             -> kejar bola (CHASE_FREE_BALL)
            if CHASE_FREE_BALL and not teammateBallReached:
                gameState = GameState.Attacking_Attacker
            else:
                gameState = GameState.Defending_Attacker

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# PRIMITIF GERAK -> command ke kinematic
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

lastTarget = None      # target terakhir (frame global) -- buat log progres


def publishTarget(x, y, theta):
    """x, y, theta dalam frame GLOBAL; dikonversi ke frame odometry buat kinematic."""
    global lastTarget
    lastTarget = [x, y, theta]
    p = Pose2D()
    p.x, p.y, p.theta = globalToOdom(x, y, theta)
    target_pub.publish(p)
    sleep(0.1)          # kasih waktu sampai ke kinematic sebelum command dikirim


def sendCommand(move):
    command_pub.publish(move.value)


def waitUntilDone(timeout=15.0, abort=None, stay=None):
    """Tunggu kinematic bilang selesai (/barracuda_kinematic/status == 0). Return True kalau benar-benar selesai.
    Keluar lebih awal HANYA kalau:
      - stay=None : enable berubah / Stop.   stay=(...) : enable keluar dari daftar itu (mis. Stop atau
        set piece lain). Enable dalam `stay` (mis. Play yang datang lebih awal) TIDAK memotong penantian.
      - timeout (detik) lewat; timeout=None -> tunggu sampai benar-benar selesai
      - abort() True"""
    enable0 = enable
    t0 = rospy.get_time()
    last_log = t0
    # handshake: kinematic langsung publish BUSY begitu terima command baru
    while not kinBusy and (rospy.get_time() - t0) < 0.3:
        if rospy.is_shutdown():
            return False
        sleep(0.02)
    while not rospy.is_shutdown():
        if not kinBusy:
            sleep(0.05)
            return True
        if stay is not None:
            if enable not in stay:
                return False
        elif enable != enable0 or enable == Enable.Stop:
            return False
        if abort is not None and abort():
            return False
        now = rospy.get_time()
        if timeout is not None and (now - t0) > timeout:
            rospy.logwarn("waitUntilDone: timeout %.1fs" % timeout)
            return False
        if timeout is None and (now - last_log) >= 5.0:
            last_log = now
            if lastTarget is not None:
                rospy.logwarn("Menuju titik... %.0f dtk, sisa jarak %.0f mm, heading beda %.0f deg"
                              % (now - t0, distance(myXY(), lastTarget[:2]),
                                 wrap180(lastTarget[2] - currentPose.theta)))
        sleep(0.05)
    return False


def waitUntilBallReached(timeout=2.0):
    """Tunggu bola ketangkap (ballReached). Berhenti SEGERA kalau juri mengganti perintah (bukan cuma Stop),
    supaya Play() bisa keluar dan roda langsung di-STOP."""
    enable0 = enable
    t0 = rospy.get_time()
    while not ballReached and not rospy.is_shutdown():
        if enable != enable0 or (rospy.get_time() - t0) > timeout:
            return False
        sleep(0.05)
    return ballReached


def waitUntilBallMoved(timeout=None):
    if timeout is None:
        timeout = AWAY_WAIT_TIMEOUT
    start = getBallPosition()
    t0 = rospy.get_time()
    enable0 = enable
    while not rospy.is_shutdown():
        if enable != enable0 or (rospy.get_time() - t0) > timeout:
            break
        now = getBallPosition()
        if start is not None and now is not None and distance(start, now) > BALL_MOVED:
            break
        sleep(0.05)
    sleep(0.1)


def gotoPose(pose, timeout=20.0, abort=None, stay=None):
    """ABSPOSE: kirim target lalu TETAP di state ini sampai kinematic melapor sampai.
    Tidak ada pembatalan karena bola terlihat dsb. (abort hanya kalau dipasang eksplisit)."""
    publishTarget(pose[0], pose[1], pose[2])
    sendCommand(Move.GotoAbsPose)
    return waitUntilDone(timeout, abort, stay)


def gotoPosition(position, timeout=15.0, abort=None):
    """Pindah ke (x, y) sambil menjaga heading sekarang."""
    return gotoPose([position[0], position[1], currentPose.theta], timeout, abort)


def setTheta(theta):
    """Putar di tempat ke `theta` (derajat). Kalau lagi pegang bola: putar sambil
    nahan bola (DRIBLE_BALL); kalau tidak: tahan posisi (GOTO_ABSPOSE)."""
    publishTarget(currentPose.x, currentPose.y, theta)
    sendCommand(Move.DribleBall if ballReached else Move.GotoAbsPose)
    return waitUntilDone(6.0)


def setTheta2Point(point):
    return setTheta(getAngle(currentPose, point))


def gotoBall():
    """Kejar bola pakai kamera kinematic sampai ketangkap dribbler."""
    sendCommand(Move.GotoBall)
    return waitUntilDone(12.0, abort=lambda: teammateBallReached or ownBallAge() > 2.0)


def WaitBall():
    """Hadap bola & nunggu operan sampai kena proximity."""
    sendCommand(Move.WaitBall)
    return waitUntilDone(6.0)


def FaceBall():
    sendCommand(Move.FaceBall)
    return waitUntilDone(5.0)


def ShootBall(angle=None):
    """Tendang penuh. angle = arah hadap tembak (derajat). Kalau None -> ke arah
    hadap sekarang. Kinematic yang muter-nahan bola, tunggu lurus, lalu nendang."""
    if enable == Enable.Stop:
        return False
    if angle is None:
        angle = currentPose.theta
    publishTarget(currentPose.x, currentPose.y, angle)
    sendCommand(Move.Shoot)
    return waitUntilDone(10.0)


def PassBall(angle=None):
    """Tendang pelan ke arah `angle` (operan ke teman)."""
    if enable == Enable.Stop:
        return False
    if angle is None:
        angle = currentPose.theta
    publishTarget(currentPose.x, currentPose.y, angle)
    sendCommand(Move.Pass)
    return waitUntilDone(10.0)


# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# TAKTIK  (logika lama)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def AdvanceAttack():
    gotoBall()                       # gerak HANYA berdasarkan state: GOTO_BALL (kinematic diam kalau bola tak terlihat)
    if not waitUntilBallReached():
        return

    # target gawang (logika lama)
    left_angle = abs(getAngle(currentPose, LEFT_GOAL))
    right_angle = abs(getAngle(currentPose, RIGHT_GOAL))
    shoot_target = LEFT_GOAL if left_angle < right_angle else RIGHT_GOAL

    # hadap target + tendang dilakukan kinematic dalam SATU command (SHOOT)
    ShootBall(getAngle(currentPose, shoot_target))


def AdvanceDefense():
    ball = getBallPosition()
    if ball is None:
        gotoPosition(DEFAULT_DEFENSE_POS)
        return

    dx = defense_goal[0] - ball[0]
    dy = defense_goal[1] - ball[1]
    norm = sqrt(dx * dx + dy * dy)
    if norm == 0:
        return

    # garis blocking antara bola dan gawang sendiri
    block_x = ball[0] + (dx / norm) * BLOCK_DIST
    block_y = ball[1] + (dy / norm) * BLOCK_DIST
    gotoPosition(clampToField(block_x, block_y))

    ball = getBallPosition() or ball

    # membayangi
    if distance(ball, defense_goal) > SAFE_RADIUS:
        retreat_x = block_x + (dx / norm) * 400
        retreat_y = block_y + (dy / norm) * 400
        gotoPosition(clampToField(retreat_x, retreat_y))
    else:
        gotoBall()

    # pressing
    ball = getBallPosition() or ball
    if distance(myXY(), ball) < INTERCEPT_DIST:
        gotoBall()


def defenderPass():
    # robot pegang bola tapi ada di area aman -> oper ke teman yang di depan
    angle = getAngle(currentPose, [teammatePose.x, teammatePose.y])
    isAttackerReady = teammateAlive() and teammatePose.y > ATTACK_READY_Y
    if isAttackerReady:
        PassBall(angle)                 # operan = tendang pelan
    else:
        setTheta(angle)                 # hadap teman sambil nahan bola, tunggu dia siap
        sleep(0.2)


def supportPoint():
    """Titik dukung: sisi lapangan BERLAWANAN dari teman (dicerminkan terhadap garis tengah lebar),
    agak maju dari garis tengah panjang (logika lama attackerPosition, sekarang di frame pojok kiri bawah).
    Ditambah jaga jarak: robot tidak saling deteksi, jadi jangan berdiri terlalu dekat teman."""
    teman_di_kanan = teammatePose.x > CX
    tx = CX - SUPPORT_LATERAL if teman_di_kanan else CX + SUPPORT_LATERAL
    ty = SUPPORT_Y
    if teammateAlive() and distance([tx, ty], [teammatePose.x, teammatePose.y]) < SUPPORT_MIN_SEPARATION:
        tx += (-1 if teman_di_kanan else 1) * SUPPORT_MIN_SEPARATION  # geser menjauh dari teman
    return clampToField(tx, ty)


def attackerPosition():
    gotoPosition(supportPoint())

    # hadap ke teman, lalu nunggu operan
    setTheta(getAngle(currentPose, [teammatePose.x, teammatePose.y]))
    WaitBall()


def state_Attacking_Attacker():
    AdvanceAttack()


def state_Attacking_Defender():
    defenderPass()


def state_Defending_Defender():
    AdvanceDefense()


def state_Defending_Attacker():
    attackerPosition()


def Play():
    while enable == Enable.OnPlay and not rospy.is_shutdown():
        updateRole()
        updateGameState()

        if gameState == GameState.Attacking_Attacker:
            state_Attacking_Attacker()
        elif gameState == GameState.Attacking_Defender:
            state_Attacking_Defender()
        elif gameState == GameState.Defending_Defender:
            state_Defending_Defender()
        elif gameState == GameState.Defending_Attacker:
            state_Defending_Attacker()

        sleep(0.05)

    # Keluar dari Play (juri kirim Stop / set piece baru): roda langsung berhenti, jangan biarkan
    # perintah terakhir (mis. GOTO_BALL) tetap jalan sampai state berikutnya sempat mengirim perintah.
    if not rospy.is_shutdown() and enable != Enable.OnPlay:
        command_pub.publish(Move.Stop.value)

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #
# REFEREE BOX (Enable)
# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

def idleWhile(state):
    while enable == state and not rospy.is_shutdown():
        sleep(0.05)


def endState(state):
    """Set Empty HANYA kalau referee belum ganti state (hindari menimpa OnPlay yang baru masuk)."""
    global enable
    if enable == state:
        enable = Enable.Empty


def waitArrived(target, stay, timeout, play_grace):
    """Tunggu robot 'sampai' di target=[x, y, heading] (frame global) dengan toleransi bertingkat.
    Return: 'arrived' (kinematic melapor sampai) | 'close' (bertahan di zona hampir-sampai, atau Play ditekan
    saat sudah di zona itu, hanya kalau PLAY_START_IN_SOFT_ZONE) | 'gave_up' (Play ditekan tapi robot belum sampai dalam PLAY_START_GRACE detik)
    | 'timeout' (RESTART_POSITION_TIMEOUT habis) | 'abort' (juri mengirim Stop / set piece lain)."""
    t0 = rospy.get_time()
    last_log = t0
    soft_since = None
    play_t = None
    while not kinBusy and (rospy.get_time() - t0) < 0.3:      # handshake: kinematic langsung BUSY
        if rospy.is_shutdown():
            return 'abort'
        sleep(0.02)
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if not kinBusy:
            sleep(0.05)
            return 'arrived'
        if enable not in stay:
            return 'abort'
        d = distance(myXY(), target[:2])
        dth = abs(wrap180(target[2] - currentPose.theta))
        in_soft = d <= ARRIVE_SOFT_POS and dth <= ARRIVE_SOFT_THETA
        if in_soft:
            if soft_since is None:
                soft_since = now
        else:
            soft_since = None
        if soft_since is not None and (now - soft_since) >= ARRIVE_SOFT_SETTLE:
            rospy.logwarn("Dianggap sampai (zona hampir-sampai %.1fs): sisa %.0f mm, heading beda %.0f deg"
                          % (ARRIVE_SOFT_SETTLE, d, dth))
            return 'close'
        if play_grace and enable == Enable.OnPlay:
            if play_t is None:
                play_t = now
            if in_soft and PLAY_START_IN_SOFT_ZONE:
                return 'close'                                 # opsi: Play sudah ditekan & sudah dekat -> mulai strategi sekarang
            if (now - play_t) >= PLAY_START_GRACE:
                rospy.logwarn("Play ditekan tapi belum sampai (%.0f s): sisa %.0f mm -> strategi mulai dari posisi sekarang"
                              % (PLAY_START_GRACE, d))
                return 'gave_up'
        if timeout is not None and (now - t0) > timeout:
            rospy.logwarn("Belum sampai titik setelah %.0f s (sisa %.0f mm) -> berhenti mencoba" % (timeout, d))
            return 'timeout'
        if (now - last_log) >= 5.0:
            last_log = now
            rospy.logwarn("Menuju titik... %.0f dtk, sisa jarak %.0f mm, heading beda %.0f deg" % (now - t0, d, dth))
        sleep(0.05)
    return 'abort'


def gotoPoseSoft(pose, stay, play_grace=False):
    """ABSPOSE dengan toleransi bertingkat (lihat waitArrived). Return status waitArrived."""
    publishTarget(pose[0], pose[1], pose[2])
    sendCommand(Move.GotoAbsPose)
    return waitArrived(pose, stay, RESTART_POSITION_TIMEOUT, play_grace)


def restartPosition(pose, en):
    """Setelah juri klik set piece `en`: ke titik referee box (toleransi bertingkat, ada batas waktu).
    Return True kalau dianggap sampai / boleh lanjut. SET_PIECE_MOVE_TO_POSE = False: tidak bergerak, robot di-STOP."""
    if SET_PIECE_MOVE_TO_POSE:
        status = gotoPoseSoft(pose, (en, Enable.OnPlay), play_grace=True)
        return status in ('arrived', 'close', 'timeout', 'gave_up')
    command_pub.publish(Move.Stop.value)
    return False


def settleForPlay(en, arrived):
    """Sudah di titik -> (WAIT_PLAY_HOLD == 'stop') STOP roda & dribbler sambil menunggu Play.
    Kalau belum sampai (timeout) robot dibiarkan terus mencoba mencapai titik."""
    if WAIT_PLAY_HOLD == 'stop' and enable == en and (arrived or not SET_PIECE_MOVE_TO_POSE):
        command_pub.publish(Move.Stop.value)


def waitWhilePlaying(cond, timeout):
    """Diam-tunggu selama OnPlay sampai cond() True (return True) atau timeout (False)."""
    t0 = rospy.get_time()
    while enable == Enable.OnPlay and not rospy.is_shutdown():
        if cond():
            return True
        if rospy.get_time() - t0 > timeout:
            return False
        sleep(0.05)
    return False


def kickerPass():
    """Kicker: ambil bola, oper ke teman, lalu (sesuai KICKER_AFTER_PASS) geser ke titik dukung
    atau diam, dan nunggu teman ambil bola."""
    if not ballVisible() and not waitWhilePlaying(ballVisible, KICKOFF_SEE_BALL_TIMEOUT):
        return           # bola tidak terlihat kamera -> main normal (tidak ada gerak mendekati bola lewat koordinat)
    gotoBall()
    if not waitUntilBallReached():
        return
    PassBall(getAngle(currentPose, [teammatePose.x, teammatePose.y]))   # tendang pelan ke teman
    if KICKER_AFTER_PASS == 'support':
        sx, sy = supportPoint()
        # menghadap teman waktu jalan; berhenti kalau juri ganti state
        gotoPose([sx, sy, getAngle(currentPose, [teammatePose.x, teammatePose.y])], timeout=8.0)
        sendCommand(Move.WaitBall)       # hadap bola, roller pelan -> siap kalau dioper balik
    else:
        sendCommand(Move.FaceBall)       # jangan langsung ngejar bola yang baru dioper
    waitWhilePlaying(lambda: teammateBallReached, KICKER_POST_PASS_WAIT)


def receiverWait():
    """Penerima: siaga di titik, hadap bola. Ambil bola (GOTO_BALL: bergerak mendekati bola pakai
    kamera, jadi operan agak melenceng tetap kekejar) begitu salah satu terjadi:
      1) bola sudah <= 1 m dari dirinya, atau
      2) bola sudah berhenti (operan melenceng, >1 m) -> tetap dijemput.
    Syarat keduanya: bola memang sudah bergeser dari posisi awal (sudah dioper) dan tidak
    sedang dipegang kicker."""
    start = getBallPosition()
    sendCommand(Move.FaceBall)
    hist = []        # (waktu, posisi bola) buat deteksi "bola berhenti"

    def ready_to_take():
        b = getBallPosition()
        if b is None or teammateBallReached:      # bola dipegang kicker -> jangan ngambil
            return False
        if start is not None and distance(start, b) < KICKOFF_MIN_BALL_TRAVEL:
            return False                          # bola belum dioper
        if distance(myXY(), b) <= RECEIVE_TRIGGER:
            return True                           # (1) sudah <= 1 m
        # (2) bola berhenti? cuma dipercaya kalau lagi TERLIHAT kamera sendiri (bukan posisi basi)
        now = rospy.get_time()
        if ballVisible():
            hist.append((now, b))
        hist[:] = [h for h in hist if now - h[0] <= BALL_STOPPED_SEC + 0.15]
        if hist and now - hist[0][0] >= BALL_STOPPED_SEC:
            return max(distance(h[1], b) for h in hist) < BALL_STOPPED
        return False

    if waitWhilePlaying(ready_to_take, KICKOFF_RECEIVE_TIMEOUT):
        gotoBall()       # GOTO_BALL (kamera)


def passFirstPlay():
    """Skenario restart kita: PASSING DULU. Sesudah ini Play() biasa."""
    if not teammateAlive():
        return                       # sendirian -> langsung main normal
    if IS_KICKER:
        kickerPass()
    else:
        receiverWait()


def homeRestart(en, pose):
    """Set piece KITA: ke titik, diam, tunggu Play. Kalau en termasuk PASS_FIRST_RESTARTS:
    setelah Play, passing dulu ke teman. Habis itu strategi biasa (Play)."""
    arrived = restartPosition(pose, en)
    settleForPlay(en, arrived)
    idleWhile(en)
    if enable == Enable.OnPlay and en in PASS_FIRST_RESTARTS:
        passFirstPlay()
    Play()


def awayRestart(en, pose):
    """Set piece LAWAN: ke titik, hadap bola, diam, tunggu Play, lalu tunggu bola
    bergeser 1 m ATAU 7 detik (mana yang duluan) baru main."""
    arrived = restartPosition(pose, en)
    if enable == en and SET_PIECE_MOVE_TO_POSE:
        FaceBall()                       # hadap bola dulu, baru diam
    settleForPlay(en, arrived)
    idleWhile(en)
    if enable == Enable.OnPlay:
        waitUntilBallMoved()
    Play()


def runReferee():
    global enable, arahPenalty
    en = enable

    if en == Enable.Stop:
        command_pub.publish(Move.Stop.value)
        rospy.loginfo("Stop")
        endState(en)

    elif en == Enable.OnPlay:
        Play()

    elif en == Enable.StartPosition:
        arrived = restartPosition(Pose.KickOffHome, en)
        settleForPlay(en, arrived)
        endState(en)

    elif en == Enable.DropBall:
        homeRestart(en, Pose.DropBall)

    elif en == Enable.KickOffHome:
        homeRestart(en, Pose.KickOffHome)

    elif en == Enable.KickOffAway:
        awayRestart(en, Pose.KickOffAway)

    elif en == Enable.GoalKickHome:
        homeRestart(en, Pose.GoalKickHome)

    elif en == Enable.GoalKickAway:
        awayRestart(en, Pose.GoalKickAway)

    elif en == Enable.FreeKickHome:
        enable = Enable.GoalKickHome

    elif en == Enable.FreeKickAway:
        enable = Enable.GoalKickAway

    elif en == Enable.CornerHome:
        arrived = restartPosition(Pose.CornerHome, en)
        settleForPlay(en, arrived)
        idleWhile(Enable.CornerHome)              # tunggu juri tekan Play
        if enable != Enable.OnPlay:               # Stop / perintah lain -> batal
            return
        gotoBall()
        gotoPoseSoft(Pose.PenaltyHome, (Enable.OnPlay,))
        ShootBall()
        gotoPoseSoft(Pose.Initial, (Enable.OnPlay,))
        endState(en)                              # sudah OnPlay -> lanjut Play() di putaran berikutnya

    elif en == Enable.CornerAway:
        awayRestart(en, Pose.CornerAway)

    elif en == Enable.PenaltyHome:
        arrived = restartPosition(Pose.PenaltyHome, en)
        settleForPlay(en, arrived)
        idleWhile(Enable.PenaltyHome)             # tunggu juri tekan Play
        if enable != Enable.OnPlay:
            return
        gotoBall()
        if arahPenalty % 2 == 0:
            target = [goal[0] + 500, goal[1]]
        elif arahPenalty % 3 == 0:
            target = [goal[0] - 500, goal[1]]
        else:
            target = [goal[0], goal[1]]
        arahPenalty += 1
        ShootBall(getAngle(currentPose, target))
        gotoPoseSoft(Pose.Initial, (Enable.OnPlay,))
        endState(en)

    elif en == Enable.PenaltyAway:
        awayRestart(en, Pose.PenaltyAway)

    elif en == Enable.ResetOdometry:
        # BUKAN mereset node odometry: robot di-STOP lalu frame lapangan dikalibrasi ulang dengan
        # anggapan robot sedang berdiri di titik Initial (sama seperti saat node strategi start).
        command_pub.publish(Move.Stop.value)
        sleep(0.3)
        calibrateFrame(Pose.Initial)
        endState(en)

    elif en == Enable.ThrowInHome:
        enable = Enable.GoalKickHome

    elif en == Enable.ThrowInAway:
        enable = Enable.GoalKickAway


# Restart kita yang dimulai dengan PASSING dulu (tambah GoalKickHome kalau mau ikut, mis.
# (Enable.KickOffHome, Enable.GoalKickHome) -> FreeKick/ThrowIn Home ikut karena dipetakan ke GoalKickHome).
PASS_FIRST_RESTARTS = (Enable.KickOffHome,)


if __name__ == '__main__':
    rospy.init_node('regional')

    robot_name = str(rospy.get_param("~robot_name", "caca")).lower()
    if robot_name not in POSES:
        rospy.logfatal("~robot_name harus 'caca' atau 'cici', dapat: %s" % robot_name)
        raise SystemExit(1)
    TABLE_YAW_SIGN = float(rospy.get_param("~yaw_sign", TABLE_YAW_SIGN))
    TABLE_YAW_OFFSET_DEG = float(rospy.get_param("~yaw_offset_deg", TABLE_YAW_OFFSET_DEG))
    ARRIVE_SOFT_POS = float(rospy.get_param("~arrive_soft_pos_mm", ARRIVE_SOFT_POS))
    ARRIVE_SOFT_THETA = float(rospy.get_param("~arrive_soft_theta_deg", ARRIVE_SOFT_THETA))
    ARRIVE_SOFT_SETTLE = float(rospy.get_param("~arrive_soft_settle_sec", ARRIVE_SOFT_SETTLE))
    PLAY_START_GRACE = float(rospy.get_param("~play_start_grace_sec", PLAY_START_GRACE))
    PLAY_START_IN_SOFT_ZONE = bool(rospy.get_param("~play_start_in_soft_zone", PLAY_START_IN_SOFT_ZONE))
    _t = rospy.get_param("~restart_position_timeout_sec", RESTART_POSITION_TIMEOUT)
    RESTART_POSITION_TIMEOUT = None if _t is None or float(_t) <= 0 else float(_t)      # <=0 : tanpa batas waktu
    setField(float(rospy.get_param("~field_width_mm", FIELD_WIDTH_MM)),
             float(rospy.get_param("~field_length_mm", FIELD_LENGTH_MM)))
    Pose = PoseTable(POSES[robot_name])
    IS_CACA = (robot_name == 'caca')
    IS_KICKER = (robot_name == str(rospy.get_param("~kickoff_kicker", KICKOFF_KICKER)).lower())
    rospy.loginfo("Strategy: %s (%s kick-off) | titik dipakai APA ADANYA (mm, tanpa skala) | ANGLE_SIGN=%s"
                  % (robot_name, "KICKER" if IS_KICKER else "PENERIMA", ANGLE_SIGN))
    rospy.loginfo("Initial=%s KickOffHome=%s" % (Pose.Initial, Pose.KickOffHome))
    rospy.loginfo("Lapangan %dx%d mm | gawang lawan=(%.0f, %.0f) gawang sendiri=(%.0f, %.0f) | yaw = %.0f*yaw%+.0f"
                  % (FIELD_WIDTH_MM, FIELD_LENGTH_MM, goal[0], goal[1], defense_goal[0], defense_goal[1],
                     TABLE_YAW_SIGN, TABLE_YAW_OFFSET_DEG))

    enable = Enable(int(rospy.get_param("~local_en", 0)))

    # self
    rospy.Subscriber('/base_station/command/data', Int8, enHandler)
    rospy.Subscriber('/robot/reset', Empty, resetHandler)
    # kinematic
    rospy.Subscriber('/robot/kinematic/odometry/pose', Pose2D, odomPoseHandler)
    rospy.Subscriber('/barracuda_kinematic/status', Int8, kinStatusHandler)
    rospy.Subscriber('/barracuda_kinematic/ball_reached', Bool, ballReachedHandler)
    rospy.Subscriber('/barracuda_kinematic/ball/global', Point, ownBallHandler)
    # teammate (relay eksternal; robot tidak saling deteksi, murni koordinat global)
    rospy.Subscriber('/caca/teammate_pose', Pose2D, teammateHandler)
    rospy.Subscriber('/caca/ball_reached', Bool, teammateBallReachedHandler)
    rospy.Subscriber('/caca/ball_global', Point, teammateBallHandler)

    command_pub.publish(Move.Stop.value)
    sleep(3)
    reset(calibrate=bool(rospy.get_param("~calibrate_on_start", True)))

    while not rospy.is_shutdown():
        runReferee()
        rospy.Rate(20).sleep()

    rospy.spin()