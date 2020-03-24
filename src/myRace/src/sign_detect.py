#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import math
import os
from time import sleep
from std_msgs.msg import String
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

pub_image = rospy.Publisher('sign_image', Image, queue_size=1)
pub_sign = rospy.Publisher('sign', String, queue_size=1)
cvBridge = CvBridge()
counter = 0
FLANN_INDEX_KDTREE = 0
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks = 50)
flann = cv2.FlannBasedMatcher(index_params, search_params)
def cbImageProjection(data):
	global kp_ideal, des_ideal, sift, counter, flann
	# drop the frame to 1/5 (6fps) because of the processing speed
	if counter % 3 != 0:
		counter += 1
		return
	cv_image_original = cvBridge.imgmsg_to_cv2(data, "bgr8")
	


	
	pub_sign.publish(sign_msg)
	pub_image.publish(cvBridge.cv2_to_imgmsg(cv_image_original, "rgb8"))
def compare_matches(kp,kp_ideal,matches):
	MATCHES_ERR = 2000
	MATCHES_DIST_MIN = 1
	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)
	if len(good) > MATCHES_DIST_MIN:
		src_pts = np.float32([kp[m.queryIdx].pt for m in good])
		dst_pts = np.float32([kp_ideal[m.trainIdx].pt for m in good])
		# print("distance matches count: ",len(good))
		mse_err = find_mse(src_pts,dst_pts)
		# print("mse_err: ", mse_err)
		if mse_err < MATCHES_ERR:
			return True
	return False
def find_mse(arr1, arr2):
	err = (arr1-arr2)**2
	sum_err = err.sum()
	size = arr1.shape[0]
	sum_err = sum_err/size
	return sum_err
def standart_signs():
	
	img1 = cv2.imread('pictures_light/sign0/sing0_ideal.JPEG',0)
	img2 = cv2.imread('pictures_light/sign1/sing1_ideal.JPEG',0)
	img3 = cv2.imread('pictures_light/sign2/sing2_ideal.JPEG',0)
	img4 = cv2.imread('pictures_light/sign3/sing3_ideal.JPEG',0)
	img4 = resize_img(img4)
	sift = cv2.xfeatures2d.SIFT_create()
	kp1,des1 = sift.detectAndCompute(img1, None)
	kp2, des2 = sift.detectAndCompute(img2,None)
	kp3, des3 = sift.detectAndCompute(img3,None)
	kp4, des4 = sift.detectAndCompute(img4,None)
	img1 = cv2.drawKeypoints(img1,kp1,None,(255,0,0),4)
	img2 = cv2.drawKeypoints(img2,kp2,None,(255,0,0),4)
	img3 = cv2.drawKeypoints(img3,kp3,None,(255,0,0),4)
	img4 = cv2.drawKeypoints(img4,kp4,None,(255,0,0),4)
	kp_ideal = [kp1,kp2,kp3,kp4]
	des_ideal = [des1,des2,des3,des4]
	return kp_ideal, des_ideal, sift#, img1
def maximum(v1,v2):
	if v1>=v2:
		return v1
	else:
		return v2
def increase_blue_and_red(img):
	h = img.shape[0]
	w = img.shape[1]
	size = (h, w, 1) 
	img_gray = np.zeros(size, np.uint8)
	pixels = []
	x = 0
	y = 0
	for pix in np.nditer(img):
		pixels.append(float(pix))
		if len(pixels) == 3:
			b = pixels[0]
			g = pixels[1]
			r = pixels[2]
			if r+g+b>0:
				v1 = float(r/(r+g+b))
				v2 = float(b/(r+g+b))
			else:
				v1 = 0
				v2 = 0
			img_gray.itemset((y,x,0),maximum(int(v1*255) , int(v2*255)))
			# print pixels
			del pixels[:]
			x+=1
		if x >= w:
			y+=1
			x = 0
	return img_gray

def resize_img(frame):
	scale_percent = 50 # percent of original size
	width = int(frame.shape[1] * scale_percent / 100)
	height = int(frame.shape[0] * scale_percent / 100)
	dim = (width, height)
	# resize image
	resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
	return(resized)

def define_what_turn(crop_img):
    # bgr = cv_image_original[y-croped_r:y+croped_r, x-croped_r:x+croped_r]
    width = crop_img.shape[1]
    left_half_mask = crop_img[:,0:width/2]
    right_half_mask = crop_img[:,width/2:width]
    count_left_white = cv2.countNonZero(left_half_mask)
    count_right_white = cv2.countNonZero(right_half_mask)
    # cv2.imshow("croped mask", crop_img)
    if count_left_white > count_right_white:
        return "right"
    else:
        return "left"

if __name__ == '__main__':
	rospy.init_node('image_projection')
	sub_image = rospy.Subscriber('/camera/image', Image, cbImageProjection, queue_size=1)
	kp_ideal, des_ideal, sift = standart_signs()
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
			# cv2.imshow('image',img1)
			# if cv2.waitKey(0) == 27:
				# cv2.destroyAllWindows()
				# break
		except KeyboardInterrupt:
			break
