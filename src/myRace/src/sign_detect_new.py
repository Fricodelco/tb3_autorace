#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import time 
from std_msgs.msg import String
from math import fabs,sqrt, pi
from dynamic_reconfigure.server import Server
from myRace.cfg import CamConfig
class Sign:
	def __init__(self):
		self.cvBridge = CvBridge()
		self.sub_gray = rospy.Subscriber('/normalized_image', Image, self.cbimg, queue_size=10)
		self.sub_gray_red = rospy.Subscriber('/normalized_image_red', Image, self.cbimg_red, queue_size=10)
		self.pub_image = rospy.Publisher('sign_image', Image, queue_size=1)
		self.pub_sign = rospy.Publisher('sign', String, queue_size=1)
		self.sub_bgr = rospy.Subscriber('/axis_videocap/image_raw', Image, self.cbimg_bgr, queue_size=10)
		
		# self.choose_line_sub = rospy.Subscriber("choosen_line", Vector3, self.cb_choose, queue_size = 2)
		img1 = cv2.imread('/home/nuc/tb_ws/src/myRace/src/ideal_signs/parking.jpg',0)
		img2 = cv2.imread('/home/nuc/tb_ws/src/myRace/src/ideal_signs/left.jpg',0)
		img3 = cv2.imread('/home/nuc/tb_ws/src/myRace/src/ideal_signs/right.jpg',0)
		img4 = cv2.imread('/home/nuc/tb_ws/src/myRace/src/ideal_signs/new.jpg',0)
		img5 = cv2.imread('/home/nuc/tb_ws/src/myRace/src/ideal_signs/road_work.jpg',0)
		self.sift = cv2.xfeatures2d.SIFT_create()
		kp1,des1 = self.sift.detectAndCompute(img1, None)
		kp2, des2 = self.sift.detectAndCompute(img2,None)
		kp3, des3 = self.sift.detectAndCompute(img3,None)
		kp4, des4 = self.sift.detectAndCompute(img4,None)
		kp5, des5 = self.sift.detectAndCompute(img5,None)
		# img1 = cv2.drawKeypoints(img1,kp1,None,(255,0,0),4)
		# img2 = cv2.drawKeypoints(img2,kp2,None,(255,0,0),4)
		# img3 = cv2.drawKeypoints(img3,kp3,None,(255,0,0),4)
		# img4 = cv2.drawKeypoints(img4,kp4,None,(255,0,0),4)
		self.kp_ideal = [kp1,kp2,kp3,kp4,kp5]
		self.des_ideal = [des1,des2,des3,des4,des5]
		self.counter = 1
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		self.flann = cv2.FlannBasedMatcher(index_params, search_params)
		self.bgr_image = np.zeros((240,320,3), np.uint8)
		self.red_gray = np.zeros((240,320,3), np.uint8)
		self.threshold = 0
		self.max_err_rect = 0
		self.max_err_ellipse = 0
		self.k_err_rect = 0
		self.k_err_ellipse = 0 
		self.srv = Server(CamConfig, self.read_param)
	def read_param(self, config, level):
		try:
			self.threshold = config["threshold"]
			self.max_err_rect = config["max_err_rect"]
			self.max_err_ellipse = config["max_err_ellipse"]
			self.k_err_rect = config["k_err_rect"]
			self.k_err_ellipse = config["k_err_ellipse"]
		except Exception as e:
			print (e)
		return config


	def cbimg_bgr(self,data):
		self.bgr_image = self.cvBridge.imgmsg_to_cv2(data, "8UC3") 
	def cbimg_red(self,data):
		self.red_gray = self.cvBridge.imgmsg_to_cv2(data, "8UC1")
	def cbimg(self,data):
		if self.counter % 4 != 0:
			self.counter += 1
			return
		else:
			self.counter = 1
		t1 = time.time()	
		img = self.cvBridge.imgmsg_to_cv2(data, "8UC1")
		# print img.shape[0]
		# img = self.resize_img(img)
		# gray = self.increase_blue_and_red(img)
		# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		ret,th1 = cv2.threshold(img,self.threshold,255,cv2.THRESH_BINARY)
		ret,th2 = cv2.threshold(self.red_gray,self.threshold,255,cv2.THRESH_BINARY)
		# kernelE = np.ones((3,3), np.uint8)
		# kernelD = np.ones((3,3), np.uint8)
		# th1 = cv2.erode(th1, kernelE, iterations=2)
		# th1 = cv2.dilate(th1, kernelD, iterations=1)
		# cv2.imshow("trash", th1)
		_, contours, hierarchy = cv2.findContours(th1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# print "here"
		sign_s = "None"
		if len(contours) != 0:
			c = max(contours, key = cv2.contourArea)
			x,y,w,h = cv2.boundingRect(c)
			
			# cv2.imshow("mask", th1)
			# cv2.imshow("orig",img)
			sign = self.bgr_image[y:y+h, x:x+w]
			mask = th1[y:y+h, x:x+w]
			cv2.rectangle(self.bgr_image,(x,y),(x+w,y+h),(0,255,0),2)
			kp,des = self.sift.detectAndCompute(sign, None)
			sign = cv2.drawKeypoints(sign,kp,None,(255,0,0),4)
			# cv2.imshow("sign", sign)
			sign_s = "None"
			mse_err = []
			for i in range(0,len(self.kp_ideal)):
				matches = self.flann.knnMatch(des,self.des_ideal[i],k=2)
				mse_err.append(self.compare_matches(kp, self.kp_ideal[i], matches))				

			min_err = mse_err[0]
			iter_min = 0
			for i in range(0, len(mse_err),1):
				if mse_err[i] < min_err:
					min_err = mse_err[i]
					iter_min = i

			if min_err < 1500:
				if iter_min == 0:
					sign_s = "parking"
				elif iter_min == 1:
					sign_s = "left"
					# sign_s = self.define_what_turn(mask)
				elif iter_min == 2:
					sign_s = "right"
					# sign_s = self.define_what_turn(mask)
				elif iter_min == 3:
					sign_s = "tunnel"
				elif iter_min == 4:
					sign_s = "road_work"
			else:
				sign_s, img = self.RecognizingBarrier(th2)
				# print sign_s
				if sign_s == "None":
					sign_s = self.detect_traffic_light(self.bgr_image, th2)
			# print sign_s
			img = self.write_text(self.bgr_image,sign_s,x,y)
			msg = String()
			msg.data = sign_s
			self.pub_sign.publish(msg)
			# hsv = cv2.cvtColor(self.bgr_image, cv2.COLOR_BGR2HSV)
			self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(self.bgr_image, "bgr8"))
			# self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(th1, "8UC1"))
			# print time.time()-t1
		# if sign_s != "None":
		print sign_s
	
	def RecognizingBarrier(self,binaryR):
		# binaryR = BinaryImColor(imRGB)
		# binaryR = BinaryImNormalize(imRGB)
		# cv.imshow("norm", binaryR)
		kernelE = np.ones((3,3), np.uint8)
		kernelD = np.ones((2,2), np.uint8)
		binaryR = cv2.erode(binaryR, kernelE, iterations=2)
		binaryR = cv2.dilate(binaryR, kernelD, iterations=1)
		_, contoursR, hierarchy = cv2.findContours( binaryR.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		k = 0
		boxList = list(); pointList = []
		xCoord = []; yCoord = []
		for cnt in contoursR:
		    rect = cv2.minAreaRect(cnt)
		    box = cv2.boxPoints(rect)
		    box = np.int0(box)
		    areaCnt = cv2.contourArea(cnt)
		    areaBox = cv2.contourArea(box)
		    if(fabs(areaBox-areaCnt)<self.k_err_rect*areaBox and areaCnt > self.max_err_rect and k<3):
		        x = int((box[0][0]-box[2][0])/2 + box[2][0])
		        y = int((box[1][1]-box[3][1])/2 + box[3][1])
		        xCoord.append(x)
		        yCoord.append(y)
		        pointList.append((x,y))
		        boxList.append(box)
			k+=1
		k = b = 0
		if (len(boxList)<2):
		    flagBarrier = "None"
		else:
		    A = np.vstack([xCoord, np.ones(len(xCoord))]).T
		    k, b = np.linalg.lstsq(A, yCoord)[0]

		    if (k < sqrt(3)/3 and k > -sqrt(3)/3):
		        flagBarrier = "bar_down"
		    else:
		        flagBarrier = "bar_up"
		img = self.DrawImage(self.bgr_image,boxList,pointList,k,b,flagBarrier)
		return flagBarrier, img	

	def detect_traffic_light(self, img, binaryR):
		kernelE = np.ones((3,3), np.uint8)
		kernelD = np.ones((3,3), np.uint8)
		binaryR = cv2.erode(binaryR, kernelE, iterations=2)
		binaryR = cv2.dilate(binaryR, kernelD, iterations=1)
		_, contoursR, hierarchy = cv2.findContours( binaryR.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		maxErr = 100000000
		maxEllips = list()
		areaEllipse = 0
		flag = "None"
		for cnt in contoursR:
		    if(len(cnt)>4):
		        (x, y), (MA, ma), angle = cv2.fitEllipse(cnt)
		        areaCnt = cv2.contourArea(cnt)
		        areaEllipse = pi / 4 * MA * ma
		        if (fabs(areaEllipse-areaCnt)<maxErr and areaCnt > self.max_err_ellipse):
		            # if img[x][y][0]+img[x][y][1]+img[x][y][2]>250:
		            	# return "None"
		            maxEllips = ((x, y), (MA, ma), angle)
		            
		            maxErr = fabs(areaEllipse-areaCnt)
		if (len(maxEllips)>0):
		    if(maxErr < self.k_err_ellipse*areaEllipse):
		        flag = "red_traffic"
		        return flag
		return flag




	def DrawImage(self,image,boxList,pointList,k,b,flagBarrier):
		for i in range(len(pointList)):
		    cv2.drawMarker(image,pointList[i],(255,255,255))
		    cv2.drawContours(image,[boxList[i]],0,(255,0,0),2)
		if (len(pointList)):
		    pointList = np.array(pointList)
		    x1 = max(pointList[:,0])
		    x2 = min(pointList[:,0])
		    y1 = int(x1*k + b)
		    y2 = int(x2*k + b)
		    cv2.line(image,(x1,y1),(x2,y2),(255,255,255))
		    if (flagBarrier == 1):
		        colorFlag = (0,255,0)
		    else:
		        colorFlag = (0,0,255)
		    cv2.circle(image,(100,100),10,colorFlag,-1)
		return image
		# for _ in range(1):
		    # cv.imshow('original',image)
		    # time.sleep(1)
		    # if (cv.waitKey(1)==ord('q')):
		        # break

	def write_text(self,img, text,x,y):
		font                   = cv2.FONT_HERSHEY_SIMPLEX
		bottomLeftCornerOfText = (x,y)
		fontScale              = 1
		fontColor              = (255,255,0)
		lineType               = 2

		cv2.putText(img,text, 
			bottomLeftCornerOfText, 
			font, 
			fontScale,
			fontColor,
			lineType)
		return img
	
	def compare_matches(self,kp,kp_ideal,matches):
		MATCHES_ERR = 500
		MATCHES_DIST_MIN = 1
		good = []
		for m,n in matches:
			if m.distance < 0.7*n.distance:
				good.append(m)
		if len(good) > MATCHES_DIST_MIN:
			src_pts = np.float32([kp[m.queryIdx].pt for m in good])
			dst_pts = np.float32([kp_ideal[m.trainIdx].pt for m in good])
			# print("distance matches count: ",len(good))
			mse_err = self.find_mse(src_pts,dst_pts)
			print("mse_err: ", mse_err)
			# if mse_err < MATCHES_ERR:
				# return True
			return mse_err
		else:
			return 10000

	def find_mse(self, arr1, arr2):
		err = (arr1-arr2)**2
		sum_err = err.sum()
		size = arr1.shape[0]
		sum_err = sum_err/size
		return sum_err


	def define_what_turn(self,crop_img):
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
    rospy.init_node('sign_detect')
    sign = Sign()
    while not rospy.is_shutdown():
        try:
            # error.calculate_error()
            rospy.sleep(0.05)
        except KeyboardInterrupt:
            break
            print("Shutting down")

        
