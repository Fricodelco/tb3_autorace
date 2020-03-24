
import numpy as np
import cv2
from time import sleep
import matplotlib.pylab as plt
cap = cv2.VideoCapture('black_line.mp4')

def matplot_show(img):
	# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	plt.imshow(img)
	plt.show()

def resize_img(frame):
	scale_percent =  8# percent of original size
	width = int(frame.shape[1] * scale_percent / 100)
	height = int(frame.shape[0] * scale_percent / 100)
	dim = (width, height)
	# resize image
	resized = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)
	return(resized)

def region_of_interest(image):
	height = image.shape[0]
	width = image.shape[1]
	for i in range(height-1, int(height*0.8), -1):
		for j in range(0, int(width*0.55),1):
			image[i][j] = 0
	return image

def draw_the_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros(shape=[img.shape[0], img.shape[1],1], dtype=np.uint8)
    iterator = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(blank_image, (x1,y1), (x2,y2), (255, 255, 255), thickness=5)
    	print("iterator:"+str(iterator)+" "+str(line))
    	iterator+=1
    	if iterator >= int(len(lines)):
    		break 
    # sleep(0.1)
    # img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)
    img = blank_image
    return img


def rotateImage(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

def gamma_correction(image, gamma = 1.0):
	invGamma = 1.0 / gamma
	table = np.array([((i / 255.0) ** invGamma) * 255
		for i in np.arange(0, 256)]).astype("uint8")
	# apply gamma correction using the lookup table
	return cv2.LUT(image, table)

def draw_line_canny(img, mask):
	res = cv2.bitwise_and(img, img, mask = mask)
	fraction_num = np.count_nonzero(mask)
	point_arr = []
	stop_flag = False
	if fraction_num > 50:
		k = 0
		jold = 0
		for i in range(mask.shape[0]-10,0,-10):
			# if stop_flag == True:
				# break
			for j in range(mask.shape[1]-1,0,-3):
				trig = False
				for t in range(i, i+10, 2):
					if(mask[t,j] > 0):
						trig = True
						point_arr.append([j,t])
						"""k+=1
						if abs(j-jold) > 1000 and k > 1:
							point_arr.pop()
							stop_flag = True
						jold = j"""
						break
				if trig == True:
					break			
		if len(point_arr) > 0:
			print len(point_arr)
			point_before = point_arr[0]
			for point in point_arr:
				res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
				point_before = point
	return res, point_arr

def draw_line(img, mask):
	fraction_num = np.count_nonzero(mask)
	point_arr = []
	stop_flag = False
	if fraction_num > 50:
		k = 0
		jold = 0
		for i in range(mask.shape[0]-30,0,-4):
			if stop_flag == True:
				break
			for j in range(mask.shape[1]-1,0,-4):
				if mask[i,j] > 0:
					point_arr.append([j,i])
					k+=1
					if abs(j-jold) > 100 and k > 1:
						point_arr.pop()
						stop_flag = True
					jold = j
					break
		if len(point_arr) > 0:
			point_before = point_arr[0]
			for point in point_arr:
				res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
				point_before = point
	return res, point_arr



if __name__ == "__main__":
	i = 0
	img = cv2.imread('ideal_signs/sign_5_ideal.jpg',1)
	print img.shape[0]
	img = resize_img(img)
	print img.shape[1]
	# img = img[0:320,0:240]
	cv2.imshow("img", img)
	cv2.imwrite("ideal_signs/sign5_ideal.JPEG", img)
	"""while(cap.isOpened()):
		try:
			ret, frame = cap.read()
			# frame =frame[0:frame.shape[0]-60, 0:frame.shape[1]]
			# resized = resize_img(frame)
			# resized = frame
			# matplot_show(frame)
			gamma_corrected = frame#gamma_correction(frame, 0.2)
			# blured = cv2.GaussianBlur(gamma_corrected, (5, 5), 0)
			# hsv = cv2.cvtColor(blured, cv2.COLOR_BGR2HSV)
			gray = cv2.cvtColor(gamma_corrected, cv2.COLOR_BGR2GRAY)
			canny_image = cv2.Canny(gray, 100, 200)
			# cropped = region_of_interest(canny_image)
			# rotated = rotateImage(canny_image, 45)
			# rotated = cv2.dilate(rotated,None,iterations=1)
			# rotated = cv2.erode(rotated,None,iterations=1)
			lines = cv2.HoughLinesP(rotated,
                        rho=4,
                        theta=np.pi/180,
                        threshold=70,
                        lines=np.array([]),
                        minLineLength=20,
                        maxLineGap=40
			try:
				print "start of new line"
				print len(lines)
				image_with_lines = draw_the_lines(blured, lines)
				
				cv2.imshow('canny_frame',canny_image)
				res, points = draw_line(blured, image_with_lines)

				# cv2.imshow('line',line)
				cv2.imshow('blured',res
			cv2.imshow("canny", canny_image)
			cv2.imshow("gamma_corrected", gamma_corrected)
			# res, points = draw_line_canny(blured, rotated)
			# cv2.imshow("result", res)
			# except Exception as e:
				# print(e)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			i+=1
			# sleep(0.05)
		except Exception as e:
			print e
			break"""

	# cap.release()
	cv2.waitKey(0)
	cv2.destroyAllWindows()
	# print("frames"+str(i))