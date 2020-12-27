import cv2
import numpy as np
from picamera import PiCamera
import picamera
import picamera.array
import time
import os
import warnings
from game import *

#manually determined
corners = ((538, 98), (2260, 76), (2274, 1778), (594, 1820))

blue_lower = (0,0,0)
blue_upper = (30,254,180)
#blue_lower = (85,55,0)
#blue_upper = (100,80,30)
orange_lower = (110,100,100)
orange_upper = (120,255,255)

def main():
    print("Starting test")
    warnings.filterwarnings("ignore")
    camera = PiCamera()
    camera.resolution = (2592, 1944)
    camera.framerate = 15
    #wait for board to be set up correctly
    print(capture_board_state(camera))

def capture_board_state(camera):
    board = np.empty((1952,2592,3), dtype=np.uint8)
    #Photograph board
    camera.capture(board, format='rgb')
    #raw camera capture produces BGR color pixels, converted to RGB using cv2
    board = cv2.cvtColor(board, cv2.COLOR_BGR2RGB)
    cv2.imwrite('raw.jpg', board)
    board = perspective_transform(board, corners)
    board = rotate(board, 90)
    cv2.imwrite('board.jpg', board)
    
    hsv_img = cv2.cvtColor(board,cv2.COLOR_RGB2HSV)
    print(hsv_img[528][342])
    blue = cv2.inRange(hsv_img, blue_lower, blue_upper) 
    orange = cv2.inRange(hsv_img, orange_lower, orange_upper)
    cv2.imwrite('orange.jpg', orange)
    cv2.imwrite('blue.jpg', blue)
    
    #produce color map based on board layout
    positions = read_board(board)
    return positions


#used to simplify testing
def board_setup_complete(positions):

    for y in range(8):
        for x in range(8):
            if y < 2:
                if positions[7-y][x] != WHITE:
                    return False
            elif y > 5:
                if positions[7-y][x] != BLACK:
                    return False
            else:
                if positions[7-y][x] != 0:
                    return False
    return True

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

