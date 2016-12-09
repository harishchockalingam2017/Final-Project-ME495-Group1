#!/usr/bin/env python
import sys
import numpy as np
import rospy
import baxter_interface as bi

#cv2
import cv2
from cv_bridge import CvBridge, CvBridgeError
import trackbar as tb

#messages
from Baxter_Catan.msg import boardcubepub
from Baxter_Catan.msg import boardtile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

def ImagTrackBar():
	win = cv2.namedWindow("MyImage")
	name = "MyImage"
	cv2.createTrackbar('H_low',name,0,255,tb.nothing)
	cv2.createTrackbar('S_low',name,164,255,tb.nothing)
	cv2.createTrackbar('V_low',name,80,255,tb.nothing)
	cv2.createTrackbar('H_high',name,180,255,tb.nothing)
	cv2.createTrackbar('S_high',name,220,255,tb.nothing)
	cv2.createTrackbar('V_high',name,255,255,tb.nothing)

def WaitForImage(data, args):
	#image subscriber
	rospy.Subscriber('/cameras/right_hand_camera/image', Image, GetBoardGraph, callback_args=(args[0]))
	ImagTrackBar()
	try:
		rospy.spin()

def GetBoardandCube(data, args):
	sample_dist = 170
	#constant depth used for 3d pos
	constant_depth = float(0)
	#initialize output variable
	output = boardcubepub()
	# CV Bridge
	bridge = args[0]
	# original image
	try:
		imgOriginal = bridge.imgmsg_to_cv2(data, "bgr8")
	# color ranges
	colors_of_tiles = [[], [], [], [], [], []]
	for i in range(0, 6):
		h_low,s_low,v_low,h_hi,s_hi,v_hi = tb.trackbar("MyImage",imgOriginal)
		lower = np.array([h_low,s_low,v_low])
		upper = np.array([h_hi,s_hi,v_hi])
		colors_of_tiles[i].append(lower)
		colors_of_tiles[i].append(upper)
	#find center points
	centerPoints = getPointsFromImage(imgOriginal)
	p_len = len(centerPoints)
	# add color and number to them
	for p in range(0, p_len):
		#init cur tile obj
		cur_tile = boardtile()
		cur_tile.x = centerPoints[p][0]
		cur_tile.y = centerPoints[p][1]

		#sample color value around that loc
		cur_color = getColorAroundPosition(img, cur_tile.x, cur_tile.y, sample_dist)

		closest_color_index = -1
		closest_color_dist = 6000000.0
		for c in range(0, 6):
			color_to_check = colors_of_tiles[c]
			#find curr dist
			hl_dif = -(colors_to_check[0] - cur_color[0]) # if pos then below range
			sl_dif = -(colors_to_check[1] - cur_color[1]) # if pos then below range
			vl_dif = -(colors_to_check[2] - cur_color[2]) # if pos then below range
			hh_dif = -(colors_to_check[3] - cur_color[0]) # if pos then above range
			sh_dif = -(colors_to_check[4] - cur_color[1]) # if pos then above range
			vh_dif = -(colors_to_check[5] - cur_color[2]) # if pos then above range
			curr_dist = (hl_dif+sl_dif+vl_dif) + (hh_dif+sh_dif+vh_dif)
			#compare
			if closest_color_dist > curr_dist:
				closest_color_dist = curr_dist
				closest_color_index = c
		#assign color to point
		cur_tile.color = closest_color_index
		#find number value
		cur_tile.numer = getNumberAtPosition(img, cur_tile.x, cur_tile.y, sample_dist)
		#append
		output.board.append(cur_tile)
	#cube logic

	#publish output
	arg[1].publish(output)

def getNumberAtPosition(img, x, y, sample_dist):
	# make black and white
	bnw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
	#init dimensions
	start_x = int((float(sample_dist) * 0.5) - x)
	start_y = int((float(sample_dist) * 0.5) - y)
	end_x = int((float(sample_dist) * 0.5) + x)
	end_y = int((float(sample_dist) * 0.5) + y)
	#output
	output = 0
	# slice out the qr code or number
	raw_num_img = img[start_x:end_x, start_y:end_y]
	# send to svm for classification
	output = ClassifyWithSVM(raw_num_img, sample_dist)
	return output

def ClassifyWithSVM(frame, dim):
	svm = cv2.SVM()
	try:
		svm.load('svm/svm_data.dat')
	except (RuntimeError, TypeError, NameError):
		img = cv2.imread('svm/Markers.png',0)
		train_cells = [np.hsplit(row,dim) for row in np.vsplit(img,dim)]
		trainData = np.float32(train_cells).reshape(-1,64)
		svm_params = dict( kernel_type = cv2.SVM_LINEAR, svm_type = cv2.SVM_C_SVC, C=2.67, gamma=5.383 )
		svm.train(trainData, responses, params=svm_params)
		svm.save('svm/svm_data.dat')
		pass
	return svm.predict(frame)

def getColorAroundPosition(img, x, y, sample_dist):
	# make hsv
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	# init dimensions
	start_x = int((float(sample_dist) * 0.5) - x)
	start_y = int((float(sample_dist) * 0.5) - y)
	end_x = int((float(sample_dist) * 0.5) + x)
	end_y = int((float(sample_dist) * 0.5) + y)
	#output color in [h, s, v]
	output = [0, 0, 0]
	#above
	above = hsv[x, start_y-1]
	#below
	below = hsv[x, end_y+1]	
	#left
	left = hsv[start_x-1, y]
	#right
	right = hsv[start_x+1, y]
	#merge             #sum					   #average
	output = int(float(above+below+left+right)*0.25)
	return output

def getPointsFromImage(img):
	img = cv2.imread(filename)
	edges = cv2.Canny(img,10,100,apertureSize=3)
	dst = cv2.cornerHarris(edges,16,5,.1)
	black_and_white = np.zeros(img.shape, np.uint8)
	black_and_white[dst>0.01*dst.max()]=[255,255,255]
	gray = cv2.cvtColor(black_and_white, cv2.COLOR_BGR2GRAY)
	cnt = cv2.findContours(gray, 1, 2)[0]
	for c in cnt:
		x,y,w,h = cv2.boundingRect(c)
		point = (x+int(.5*w),y+int(.5*h))
		points.append(point)
	#sort points by y with thresh
	y_thresh = 2.0
	sorted_points_by_y = []
	for p in range(0, len(points)):
		point = points[p]
		# check against already sorted
		b = False
		for q in range(0, len(sorted_points_by_y)):
			for r in range(0, len(sorted_points_by_y[q])):
				if np.abs(float(point[1])-float(sorted_points_by_y[q][r][1])) < y_thresh:
					sorted_points_by_y[q].append(point)
					b = True
					break
			if b:
				break
		#if still not added, make new category and add
		if not b:
			sorted_points_by_y.append([point])
	points = []
	for s in range(0, len(sorted_points_by_y)):
		if len(sorted_points_by_y[s]) < 2:
			np.delete( sorted_points_by_y, s)
			--s
		else:
			for p in range( 0, len(sorted_points_by_y[s])):
				points.append(sorted_points_by_y[s][p])
	#old fasion sort of points by x value
	sorted_points_tmp = [points[0]]
	for p in range(1, len(points)):
		for q in range(0, len(sorted_points_tmp)):
			if points[p][0] < sorted_points_tmp[q][0]:
				sorted_points_tmp.insert(q, points[p])
				break
			elif q == len(sorted_points_tmp)-1: # append if this was last item
				sorted_points_tmp.append(points[p])
				break
	points = sorted_points_tmp
	print points
	#sort points by x with thresh
	x_thresh = 25
	sorted_points_by_x = []
	for p in range(0, len(points)):
		point = points[p]
		# check against already sorted
		b = False
		for q in range(0, len(sorted_points_by_x)):
			for r in range(0, len(sorted_points_by_x[q])):
				if np.abs(np.abs(float(point[0]))-np.abs(float(sorted_points_by_x[q][r][0]))) < x_thresh:
					sorted_points_by_x[q].append(point)
					b = True
					break
			if b:
				break
		#if still not added, make new category and add
		if not b:
			sorted_points_by_x.append([point])
	points = []
	even = True
	for s in range(0, len(sorted_points_by_x)):
		if len(sorted_points_by_x[s]) < 2:
			np.delete( sorted_points_by_x, s)
			--s
		elif even:
			np.delete( sorted_points_by_x, s)
			--s
			even = False
		else:
			even = True
			for p in range( 0, len(sorted_points_by_x[s])):
				# add z value
				points.append([sorted_points_by_x[s][p][0], sorted_points_by_x[s][p][1], z)
	return points

def main(args):
	#init node
	rospy.init_node('board_and_cube')

	### should be in launch file
	bridge = CvBridge()
	rightcam = bi.CameraController('right_hand_camera')
	###

	bridge = CvBridge()

	# final publisher
	pub = rospy.Publisher('BoardCubePub', boardcubepub, queue_size=10)
	# init subscriber to listen for request to get board
	rospy.Subscriber('BoardCubeSub', Empty, WaitForImage(), callback_args=(bridge, pub))
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)