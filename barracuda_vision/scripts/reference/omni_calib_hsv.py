#!/usr/bin/env python3
import rospy
import sys
from time import sleep
from enum import Enum
from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from barracuda_vision.msg import ballInfo, ballTravel
from filtercam import filter_cam
from dynamic_reconfigure.server import Server
from barracuda_vision.cfg import omni_paramConfig
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import detectfunc as df
import cv2
import numpy as np
from threading import Thread
#--GLOBAL VARIABEL--#
exposure_val = True
blur_val = 1
offset_W = 0
offset_H = 0
Hue_min = 0
Hue_max = 0
Sat_min = 0
Sat_max = 0
Val_min = 0
Val_max = 0

line_detect = True
Hue_min_line = 0
Hue_max_line = 0
Sat_min_line = 0
Sat_max_line = 0
Val_min_line = 0
Val_max_line = 0

slopeToggle = True
line_v1 = 0
line_v2 = 0
length_v = 0
line_h1 = 0
line_h2 = 0
length_h = 0
slopeline = 0

dummyToggle = True
Hue_min_dummy = 0
Hue_max_dummy = 0
Sat_min_dummy = 0
Sat_max_dummy = 0
Val_min_dummy = 0
Val_max_dummy = 0
crop_side_dummy = 0
crop_top_dummy = 0
crop_bottom_dummy = 0





#--CLASS VIDEO (PENGATURAN VIDEO)--#
class VideoStream:
	def __init__(self, src=0, exposure_val=0, width = 640, height = 480):
		self.stream = cv2.VideoCapture(src)

		self.stream.set(cv2.CAP_PROP_FPS, 30)
		self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
		self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
		self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, height)
		self.frame_width = int(self.stream.get(cv2.CAP_PROP_FRAME_WIDTH))
		self.frame_height = int(self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
		rospy.loginfo((self.frame_width, self.frame_height))
		self.fps = self.stream.get(cv2.CAP_PROP_FPS)
		rospy.loginfo(self.fps)

		if self.stream.get(cv2.CAP_PROP_EXPOSURE) is not None:
			self.exposure_val = self.stream.get(cv2.CAP_PROP_EXPOSURE)
			self.exposure_val = exposure_val
			self.stream.set(cv2.CAP_PROP_EXPOSURE, self.exposure_val)
		if self.stream.get(cv2.CAP_PROP_AUTOFOCUS) is not None:
			self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0)

		(self.grabbed, self.frame) = self.stream.read()
		self.stopped = False

	def start(self):
		Thread(target=self.update, args=()).start()
		return self

	def update(self):
		while True:
			if self.stopped:
				return
			(self.grabbed, self.frame) = self.stream.read()

	def read(self):
		return self.frame

	def stop(self):
		self.stopped = True
		self.stream.release()

def callbackConfig(config, level):
	global exposure_val, blur_val, offset_W, offset_H, Hue_min, Hue_max, Sat_min, Sat_max, Val_min, Val_max
	global line_detect, Hue_min_line, Hue_max_line, Sat_min_line, Sat_max_line, Val_min_line, Val_max_line
	global slopeToggle, line_v1, line_v2, length_v, line_h1, line_h2, length_h, slopeline
	global dummyToggle, Hue_min_dummy, Hue_max_dummy, Sat_min_dummy, Sat_max_dummy, Val_min_dummy, Val_max_dummy
	global crop_side_dummy, crop_top_dummy, crop_bottom_dummy

	# Update variabel global dengan nilai dari config
	exposure_val = config.exposure_val
	blur_val = config.blur_val
	offset_W = config.offset_W
	offset_H = config.offset_H
	Hue_min = config.Hue_min
	Hue_max = config.Hue_max
	Sat_min = config.Sat_min
	Sat_max = config.Sat_max
	Val_min = config.Val_min
	Val_max = config.Val_max

	line_detect = config.line_detect
	Hue_min_line = config.Hue_min_line
	Hue_max_line = config.Hue_max_line
	Sat_min_line = config.Sat_min_line
	Sat_max_line = config.Sat_max_line
	Val_min_line = config.Val_min_line
	Val_max_line = config.Val_max_line

	slopeToggle = config.slopeToggle
	line_v1 = config.line_v1
	line_v2 = config.line_v2
	length_v = config.length_v
	line_h1 = config.line_h1
	line_h2 = config.line_h2
	length_h = config.length_h
	slopeline = config.slopeLine

	dummyToggle = config.dummyToggle
	Hue_min_dummy = config.Hue_min_dummy
	Hue_max_dummy = config.Hue_max_dummy
	Sat_min_dummy = config.Sat_min_dummy
	Sat_max_dummy = config.Sat_max_dummy
	Val_min_dummy = config.Val_min_dummy
	Val_max_dummy = config.Val_max_dummy

	crop_side_dummy = config.crop_side_dummy
	crop_top_dummy = config.crop_top_dummy
	crop_bottom_dummy = config.crop_bottom_dummy

    # Simpan parameter ke rosparam
	rospy.set_param("~exposure_val", exposure_val)
	rospy.set_param("~blur_val", blur_val)
	rospy.set_param("~offset_W", offset_W)
	rospy.set_param("~offset_H", offset_H)
	rospy.set_param("~Hue_min", Hue_min)
	rospy.set_param("~Hue_max", Hue_max)
	rospy.set_param("~Sat_min", Sat_min)
	rospy.set_param("~Sat_max", Sat_max)
	rospy.set_param("~Val_min", Val_min)
	rospy.set_param("~Val_max", Val_max)

	rospy.set_param("~line_detect", line_detect)
	rospy.set_param("~Hue_min_line", Hue_min_line)
	rospy.set_param("~Hue_max_line", Hue_max_line)
	rospy.set_param("~Sat_min_line", Sat_min_line)
	rospy.set_param("~Sat_max_line", Sat_max_line)
	rospy.set_param("~Val_min_line", Val_min_line)
	rospy.set_param("~Val_max_line", Val_max_line)

	rospy.set_param("~slopeToggle", slopeToggle)
	rospy.set_param("~line_v1", line_v1)
	rospy.set_param("~line_v2", line_v2)
	rospy.set_param("~length_v", length_v)
	rospy.set_param("~line_h1", line_h1)
	rospy.set_param("~line_h2", line_h2)
	rospy.set_param("~length_h", length_h)
	rospy.set_param("~slopeLine", slopeline)

	rospy.set_param("~dummyToggle", dummyToggle)
	rospy.set_param("~Hue_min_dummy", Hue_min_dummy)
	rospy.set_param("~Hue_max_dummy", Hue_max_dummy)
	rospy.set_param("~Sat_min_dummy", Sat_min_dummy)
	rospy.set_param("~Sat_max_dummy", Sat_max_dummy)
	rospy.set_param("~Val_min_dummy", Val_min_dummy)
	rospy.set_param("~Val_max_dummy", Val_max_dummy)

	rospy.set_param("~crop_side_dummy", crop_side_dummy)
	rospy.set_param("~crop_top_dummy", crop_top_dummy)
	rospy.set_param("~crop_bottom_dummy", crop_bottom_dummy)

	rospy.loginfo(
		"Reconfigure Request: "
		"exposure_val={exposure_val}, "
		"blur_val={blur_val}, "
		"offset_W={offset_W}, "
		"offset_H={offset_H}, "
		"Hue_min={Hue_min}, "
		"Hue_max={Hue_max}, "
		"Sat_min={Sat_min}, "
		"Sat_max={Sat_max}, "
		"Val_min={Val_min}, "
		"Val_max={Val_max}, "
		"line_detect={line_detect}, "
		"Hue_min_line={Hue_min_line}, "
		"Hue_max_line={Hue_max_line}, "
		"Sat_min_line={Sat_min_line}, "
		"Sat_max_line={Sat_max_line}, "
		"Val_min_line={Val_min_line}, "
		"Val_max_line={Val_max_line}, "
		"slopeToggle={slopeToggle}, "
		"line_v1={line_v1}, "
		"line_v2={line_v2}, "
		"length_v={length_v}, "
		"line_h1={line_h1}, "
		"line_h2={line_h2}, "
		"length_h={length_h}, "
		"slopeLine={slopeLine}, "
		"dummyToggle={dummyToggle}, "
		"Hue_min_dummy={Hue_min_dummy}, "
		"Hue_max_dummy={Hue_max_dummy}, "
		"Sat_min_dummy={Sat_min_dummy}, "
		"Sat_max_dummy={Sat_max_dummy}, "
		"Val_min_dummy={Val_min_dummy}, "
		"Val_max_dummy={Val_max_dummy}, "
		"crop_side_dummy={crop_side_dummy}, "
		"crop_top_dummy={crop_top_dummy}, "
		"crop_bottom_dummy={crop_bottom_dummy}".format(**config)
	)

	return config

def logConfig():
    rospy.loginfo(
        "Processing image with: "
        "exposure_val={}, blur_val={}, Hue_min={}, Hue_max={}, "
        "Sat_min={}, Sat_max={}, Val_min={}, Val_max={}, "
        "line_detect={}, slopeToggle={}, dummyToggle={}".format(
            exposure_val,
            blur_val,
            Hue_min,
            Hue_max,
            Sat_min,
            Sat_max,
            Val_min,
            Val_max,
            line_detect,
            slopeToggle,
            dummyToggle
        )
    )
#--MAIN FUNCTION --#
if __name__ == '__main__':
	rospy.init_node('omni_cam')

	cam_index = rospy.get_param("~cam_index", 0)
	exposure_val = rospy.get_param("~exposure_val",True)
	vs = VideoStream(src=cam_index, exposure_val=exposure_val).start()

	frame_width = vs.frame_width
	frame_height = vs.frame_height

	flip_frame = rospy.get_param("~flip_frame", False)
	flip_code = rospy.get_param("~flip_code", 1)

	blur_val = rospy.get_param("~blur_val", 1)
	offset_W = rospy.get_param("~offset_W", 0)
	offset_H = rospy.get_param("~offset_H", 0)
	cam_center = (frame_width // 2 + offset_W, frame_height // 2 - offset_H)

	Hue_min = rospy.get_param("~Hue_min", 0)
	Hue_max = rospy.get_param("~Hue_max", 0)
	Sat_min = rospy.get_param("~Sat_min", 0)
	Sat_max = rospy.get_param("~Sat_max", 0)
	Val_min = rospy.get_param("~Val_min", 0)
	Val_max = rospy.get_param("~Val_max", 0)

	line_detect = rospy.get_param("~line_detect", 0)
	Hue_min_line = rospy.get_param("~Hue_min_line", 0)
	Hue_max_line = rospy.get_param("~Hue_max_line", 0)
	Sat_min_line = rospy.get_param("~Sat_min_line", 0)
	Sat_max_line = rospy.get_param("~Sat_max_line", 0)
	Val_min_line = rospy.get_param("~Val_min_line", 0)
	Val_max_line = rospy.get_param("~Val_max_line", 0)

	slopeToggle = rospy.get_param("~slopeToggle", 0)
	line_v1 = rospy.get_param("~line_v1", 0)
	line_v2 = rospy.get_param("~line_v2", 0)
	length_v = rospy.get_param("~length_v", 0)
	line_h1 = rospy.get_param("~line_h1", 0)
	line_h2 = rospy.get_param("~line_h2", 0)
	length_h = rospy.get_param("~length_h", 0)
	slopeLine = rospy.get_param("~slopeLine", 0)

	dummyToggle = rospy.get_param("~dummyToggle", 0)
	Hue_min_dummy = rospy.get_param("~Hue_min_dummy", 0)
	Hue_max_dummy = rospy.get_param("~Hue_max_dummy",0)
	Sat_min_dummy = rospy.get_param("~Sat_min_dummy",0)
	Sat_max_dummy = rospy.get_param("~Sat_max_dummy",0)
	Val_min_dummy = rospy.get_param("~Val_min_dummy",0)
	Val_max_dummy = rospy.get_param("~Val_max_dummy",0)

	crop_side_dummy = rospy.get_param("~crop_side_dummy", 0)
	crop_top_dummy = rospy.get_param("~crop_top_dummy", 0)
	crop_bottom_dummy = rospy.get_param("~crop_bottom_dummy", 0)

	server = Server(omni_paramConfig, callbackConfig)

	# Perbarui konfigurasi server dengan nilai dari global_var
	server.update_configuration({
		"exposure_val": exposure_val,
		"blur_val": blur_val,
		"offset_W": offset_W,
		"offset_H": offset_H,
		"Hue_min": Hue_min,
		"Hue_max": Hue_max,
		"Sat_min": Sat_min,
		"Sat_max": Sat_max,
		"Val_min": Val_min,
		"Val_max": Val_max,
		"line_detect": line_detect,
		"Hue_min_line": Hue_min_line,
		"Hue_max_line": Hue_max_line,
		"Sat_min_line": Sat_min_line,
		"Sat_max_line": Sat_max_line,
		"Val_min_line": Val_min_line,
		"Val_max_line": Val_max_line,
		"slopeToggle": slopeToggle,
		"line_v1": line_v1,
		"line_v2": line_v2,
		"length_v": length_v,
		"line_h1": line_h1,
		"line_h2": line_h2,
		"length_h": length_h,
		"slopeLine": slopeline,
		"dummyToggle": dummyToggle,
		"Hue_min_dummy": Hue_min_dummy,
		"Hue_max_dummy": Hue_max_dummy,
		"Sat_min_dummy": Sat_min_dummy,
		"Sat_max_dummy": Sat_max_dummy,
		"Val_min_dummy": Val_min_dummy,
		"Val_max_dummy": Val_max_dummy,
		"crop_side_dummy": crop_side_dummy,
		"crop_top_dummy": crop_top_dummy,
		"crop_bottom_dummy": crop_bottom_dummy
	})

	rate = rospy.Rate(50)
	filter = filter_cam()
	while not rospy.is_shutdown():
		frame = vs.read()
		if not vs.grabbed: break
		if flip_frame:
			filter.flipFrame(frame,flip_code)
		frame_ui = filter.blurFilter(frame,1)
		blur = filter.blurFilter(frame, blur_val)
		masking = filter.mask(blur, Hue_min, Sat_min, Val_min, Hue_max, Sat_max, Val_max)
		contour, hierarcy = filter.contourDetect(masking,0,1)
		(x,y), radius, mask = df.detect_ballEC(frame, Hue_min, Sat_min, Val_min, Hue_max, Sat_max, Val_max)
		cv2.imshow("camera", frame)
		cv2.waitKey(1)
		rospy.spin()
		cv2.destroyAllWindows()
