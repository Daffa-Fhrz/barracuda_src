#!/usr/bin/env python3
import rospy
from time import sleep
from enum import Enum
from math import sqrt, degrees, radians, sin, cos, atan2
from std_msgs.msg import Empty, Bool, Byte, Int8, Float64
from barracuda_vision.msg import ballInfo, ballTravel
from filtercam import filter_cam
from dynamic_reconfigure.server import Server
import detectfunc as df
import cv2
import numpy as np
from threading import Thread

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

	Hue_min = rospy.get_param("~Hue_min", 15)
	Hue_max = rospy.get_param("~Hue_max", 179)
	Sat_min = rospy.get_param("~Sat_min", 150)
	Sat_max = rospy.get_param("~Sat_max", 255)
	Val_min = rospy.get_param("~Val_min", 100)
	Val_max = rospy.get_param("~Val_max", 255)

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
 
	
	rate = rospy.Rate(50)
	filter = filter_cam()
	try:
		while not rospy.is_shutdown():
			# logConfig()
			frame = vs.read()
			# if not vs.grabbed: break
			# if flip_frame:
			# 	filter.flipFrame(frame,flip_code)
			frame_ui = cv2.GaussianBlur(frame, (5, 5), 0)
			hsv = cv2.cvtColor(frame_ui, cv2.COLOR_BGR2HSV)

			Hue_min = cv2.getTrackbarPos('Hue Min', 'Masking')
			Hue_max = cv2.getTrackbarPos('Hue Max', 'Masking')
			Sat_min = cv2.getTrackbarPos('Sat Min', 'Masking')
			Sat_max = cv2.getTrackbarPos('Sat Max', 'Masking')
			Val_min = cv2.getTrackbarPos('Val Min', 'Masking')
			Val_max = cv2.getTrackbarPos('Val Max', 'Masking')

			mask = cv2.inRange(hsv, (Hue_min, Sat_min, Val_min), (Hue_max, Sat_max, Val_max))
			masked_frame = cv2.bitwise_and(frame, frame, mask=mask)

			cv2.imshow("Original Frame", frame)
			cv2.imshow("HSV Frame", hsv)
			cv2.imshow("Mask", mask)
			cv2.imshow("Masked Frame", masked_frame)
		vs.stop()
		cv2.destroyAllWindows()	
	except rospy.ROSInterruptException:
		pass