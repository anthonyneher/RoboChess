import chess
import chess.engine
import numpy as np
from copy import copy
import cv2
from picamera import PiCamera
import picamera
import picamera.array
import time
import os
import warnings

#manually determined
corners = ((517, 119), (2235, 111), (2233, 1791), (565, 1835))

def main():
    #initialize python-chess board that will be used to determine response
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci("/Users/anthonyneher/senior/stockfish-11-mac/src/stockfish")
    #ignore warnings created due to 
    warnings.filterwarnings("ignore")

	camera = PiCamera()
	camera.resolution = (2592, 1944)
	camera.framerate = 15

    #wait for board to be set up correctly
    current_state = capture_board_state()
    while not board_setup_complete(current_state):
        current_state = capture_board_state()

    #Send START command

    #Entering main loop
    while True:
        #Photograph board and decode state
        previous_state = current_state
        current_state = capture_board_state()
        
        ret = novel_state(previous_state, current_state)

        if ret == 1:
            move = decode_move(board, current_state)
            if move != None:
                board.push(move)
                result = engine.play(board, chess.engine.Limit(time=0.5))
                board.push(result.move)
                pickup = move.from_square
                place = move.to_square


            else:
                #send error signal
        else:

