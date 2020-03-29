import cv2
import numpy as np
from picamera import PiCamera
import picamera
import picamera.array
import time
import chess
import os
import warnings

blue_lower = (0,120,70)
blue_upper = (20,255,255)
orange_lower = (110,100,100)
orange_upper = (120,255,255)

def main():
	warnings.filterwarnings("ignore")
	print("started")
	camera = PiCamera()
	camera.resolution = (2592, 1944)
	camera.framerate = 15
	output = np.empty((1952,2592,3), dtype=np.uint8)
	camera.capture(output, format='rgb')
	output = cv2.cvtColor(output,cv2.COLOR_BGR2RGB)
	"""
	board_size = (7,7)
	board = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
	#finding the inner 7x7 corners in the board
	found, inner_corners = cv2.findChessboardCorners(output, board_size)
	if(found): print("corners found")
	else: print("corners not found")
	"""
	#hardcoded outer corners of chess board measured using external software
	corners = ((517, 119), (2235, 111), (2233, 1791), (565, 1835))
	board = perspective_transform(output, corners)
	board = rotate(board, 90)
	cv2.imwrite('board.jpg', board)
	"""
	hsv_img = cv2.cvtColor(board,cv2.COLOR_RGB2HSV)
	blue = cv2.inRange(hsv_img, blue_lower, blue_upper)
	cv2.imwrite('blue.jpg', blue)
	
	orange = cv2.inRange(hsv_img, orange_lower, orange_upper)
	cv2.imwrite('orange.jpg',orange)
	"""
	positions = read_board(board)
	print(positions)
	
	
#rotates image by degrees specified in angle parameter
def rotate(image, angle):
	(h, w) = image.shape[:2]
	c = (w / 2, h / 2)
	m = cv2.getRotationMatrix2D(c, angle, 1.0)
	new = cv2.warpAffine(image, m, (h, w))
	return new
	
#returns subsquare image at specified index
def subsquare(image, index):
	x_size, y_size, trash = image.shape
	x = x_size/8
	y = y_size/8
	subsquare = image[int(x*index[1]):int(x*(index[1]+1)), int(y*index[0]):int(y*(index[0]+1)), :]
	return subsquare
	
#returns target window of subsquare
def target_square(image):
	x, y, trash = image.shape
	target = image[int(x/4):int(x*3/4), int(y/4):int(y*3/4), :]
	return target

def read_board(board):
	positions = np.zeros((8,8),dtype=int)
	for x in range(8):
		for y in range(8):
			square = subsquare(board, (y,x))
			target = target_square(square)
			positions[x][y] = classify(target)
	return positions


"""
Classify attempts to detect the presence of either orange or blue pieces within a square fed into it.
This is done by converting the image into the HSV color spectrum and then creating a mask of the pixels
that are within the color range specified for blue and orange. If the pixel exists, it's mask value is
set to 255 and 0 otherwise. The sum of the mask values is used to determine if piece is present
"""
def classify(square):
	threshold = 1000
	hsv_img = cv2.cvtColor(square,cv2.COLOR_RGB2HSV)
	if np.sum(cv2.inRange(hsv_img, blue_lower, blue_upper)) > threshold:
		return 2
	if np.sum(cv2.inRange(hsv_img, orange_lower, orange_upper)) > threshold:
		return 1
	else:
		return 0

def perspective_transform(src_image, corners):
	(top_left, top_right, bottom_right, bottom_left) = corners

	top_width = np.sqrt((top_right[0] - top_left[0])**2 + (top_right[1] - top_left[1])**2)
	bottom_width = np.sqrt((bottom_right[0] - bottom_left[0])**2 + (bottom_right[1] - bottom_left[1])**2)
	max_width = max(int(top_width), int(bottom_width))

	left_height = np.sqrt((bottom_left[0] - top_left[0])**2 + (bottom_left[1] - top_left[1])**2)
	right_height = np.sqrt((bottom_right[0] - top_right[0])**2 + (bottom_right[1] - top_right[1])**2)
	max_height = max(int(left_height), int(right_height))

	destination = np.array([[0,0], [max_width-1, 0], [max_width-1, max_height-1], [0, max_height-1]])

	matrix = cv2.getPerspectiveTransform(np.float32(corners), np.float32(destination))

	new_image = cv2.warpPerspective(src_image, matrix, (max_width, max_height))

	return new_image


if __name__ == '__main__':
	main()
