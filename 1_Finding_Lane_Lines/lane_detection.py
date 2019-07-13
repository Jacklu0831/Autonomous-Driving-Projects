import cv2
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys


ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", required=True,
	help="path to video input")
ap.add_argument("-o", "--output", required=True,
	help="path to video output")
args = vars(ap.parse_args())


def canny(image):
	# computes a lot of gradients and output the big ones with white pixels
	gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
	blur = cv2.GaussianBlur(gray, (5,5), 0)
	canny = cv2.Canny(blur, 50, 150)
	return canny


def region_of_interest(image):
	# define the area that we are interested in and crop the part out
	height = image.shape[0]
	polygons = np.array([[[200,height],[1100,height],[550,250]]]) # triangle
	mask = np.zeros_like(image)
	cv2.fillPoly(mask, polygons, (255,255,255))
	masked_image = cv2.bitwise_and(image, mask)
	return masked_image


def average_slope_intercept(image, lines, left_fit_prev, right_fit_prev):
	# left line has positive, right line has negative slope (0,0 is top left)
	left_fit = []
	right_fit = []
	for line in lines:
		x1, y1, x2, y2 = line.reshape(4)
		parameters = np.polyfit((x1,x2),(y1,y2),1) # fit dots degree 1, 2D array, slope and intercept
		slope, intercept = parameters
		# print("Slope = {}, intercept = {}".format(slope,intercept))
		if slope < 0:
			left_fit.append((slope,intercept))
		else:
			right_fit.append((slope,intercept))

	# quick fix, use cached previous detections to fill in the gap
	if not len(left_fit):
		left_fit = left_fit_prev
	if not len(right_fit):
		right_fit = right_fit_prev

	left_fit_average  = np.average(left_fit, axis=0)
	right_fit_average = np.average(right_fit, axis=0)
	left_fit_prev = left_fit
	right_fit_prev = right_fit
	left_line  = make_coordinates(image, left_fit_average)
	right_line = make_coordinates(image, right_fit_average)
	averaged_lines = [left_line, right_line]
	return averaged_lines, left_fit_prev, right_fit_prev


def make_coordinates(image, line_parameters):
	slope, intercept = line_parameters
	y1 = image.shape[0] # both lines start from y1 bottom to 60% up
	y2 = int(y1*0.6)
	x1 = int((y1-intercept)/slope) # x = (y-b)/m
	x2 = int((y2-intercept)/slope)
	return np.array([x1,y1,x2,y2])


def display_lines(image, lines):
	# make a image of black background and blue lines
	line_image = np.zeros_like(image)
	if lines is not None:
		for line in lines:
			x1,y1,x2,y2 = line
			cv2.line(line_image, (x1,y1), (x2,y2), (255,0,0), 10) # 10 is thickness
	return line_image


# initialize video stream
video_stream = cv2.VideoCapture(args["input"])
_, frame = video_stream.read()
fourcc = cv2.VideoWriter_fourcc(*"mp4v") # or change to *"MJPG")
writer = cv2.VideoWriter(args["output"], fourcc, 30, (frame.shape[1], frame.shape[0]), True) # videoWriter object
left_fit_prev = []
right_fit_prev = []

while(video_stream.isOpened()):
	grabbed, frame = video_stream.read()
	if grabbed is not True:
		break
	canny_image = canny(frame)
	cropped_image = region_of_interest(canny_image)
	# specify bin sizes, 2 pixels and 1 degree in radian, and threshold of 100, empty array as place holder
	# only line > 40 are lines and only points less than 5 pixels apart can be connected
	lines = cv2.HoughLinesP(cropped_image, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)
	average_lines, left_fit_prev, right_fit_prev = average_slope_intercept(frame, lines, left_fit_prev, right_fit_prev)
	line_image = display_lines(frame, average_lines)
	# why the background is black, 80% intensity of lane_image and 100% for line_image and add 0 to all pixel
	combo_image = cv2.addWeighted(frame, 0.8, line_image, 1, 0)
	writer.write(combo_image)


video_stream.release()
writer.release()



