import cv2
import numpy as np
from picamera import PiCamera
import time


def main():
	
	camera = PiCamera()
	#camera.rotation = 180
	camera.capture('raw_board.jpg')
	image = cv2.imread('raw_board.jpg')
	"""
	board_size = (7,7)
	board = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#finding the inner 7x7 corners in the board
	found, inner_corners = cv2.findChessboardCorners(image, board_size)
	"""
	
	#hardcoded outer corners of chess board measured using external software
	#corners = np.zeros((4,2), dtype = "float32")
	corners = ((23, 10), (490, 14), (490, 479), (22, 480))

	board = perspective_transform(image, corners)

	cv2.imwrite('board.jpg', board)
	
#returns subsquare image at specified index
def subsquare(image, index):
	x_size, y_size, trash = image.shape
	step = x_size/8
	x = (x_size/index[0])

	subsquare = image[int(x_size/index[0]):int(x_size/(index[0]+1)), int(y_size/index[1]):int(y_size/(index[1]+1)), :]
	return subsquare

def read_board(board):
	positions = np.zeros((8,8),dtype=int)
	for x in range(8):
		for y in range(8):
			square = subsquare(board, (y,x))
			positions[x][y] = classify(square)
	return positions


"""
Classify attempts to detect the presence of either orange or blue pieces within a square fed into it.
This is done by converting the image into the HSV color spectrum and then creating a mask of the pixels
that are within the color range specified for blue and orange. If the pixel exists, it's mask value is
set to 255 and 0 otherwise. The sum of the mask values is used to determine if piece is present
"""
def classify(square):
	min_thresh = 6.0
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
