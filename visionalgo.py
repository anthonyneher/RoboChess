import cv2
import numpy as np


print("OpenCV version:")
print(cv2.__version__)

print("Attempting to find corners of chessboard")


def main():
	#import image from desktop
	image = cv2.imread("sendtopi.png")
	board_size = (7,7)
	board = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	#finding the inner 7x7 corners in the board
	found, inner_corners = cv2.findChessboardCorners(image, board_size)

	if(not found):
		print("Unable to find board in image")

	for i in inner_corners:
		print(i)

	#testing = cv2.drawChessboardCorners(image, board_size, inner_corners, found)
	#cv2.imwrite('board.jpg', testing)

	#hardcoded outer corners of chess board measured using external software
	#corners = np.zeros((4,2), dtype = "float32")
	corners = ((23, 10), (490, 14), (490, 479), (22, 480))

	just_board = perspective_transform(image, corners)

	cv2.imwrite('just_board.jpg', just_board)
	print(just_board.shape)

#	cv2.imwrite('subsquare.jpg', subsquare(just_board,(3,5)))

#((426,106), (2163,96),(2153,1810),(480,1826))

#def find_board_state(transformed_image):

def subsquare(image, index):
	#board_size = (7,7)
	# found, inner_corners = cv2.findChessboardCorners(image, board_size)
	# testing = cv2.drawChessboardCorners(image, board_size, inner_corners, found)
	x_size, y_size, trash = image.shape
	step = x_size/8
	x = (x_size/index[0])

	subsquare = image[int(x_size/index[0]):int(x_size/(index[0]+1)), int(y_size/index[1]):int(y_size/(index[1]+1)), :]
	return subsquare


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
