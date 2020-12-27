import chess
import chess.engine
import numpy as np
import cv2
import picamera
import picamera.array
import time
import os
import warnings
from copy import copy
from picamera import PiCamera
from vision import *
from game import *
from comms import *
  
START = 0x00;
MOVE = 0x01;
DONE_YET = 0x02;
ANY_NEW = 0x03;
BOARD_DISPLAY = 0x04;


def main():
    print("Start")
    #initialize python-chess board that will be used to determine response
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci("/usr/games/stockfish")
    #ignore warnings created due to 
    warnings.filterwarnings("ignore")
    camera = PiCamera()
    camera.resolution = (2592, 1944)
    camera.framerate = 15
    spi = init_spi()
    
    displayed = 0
    prevdisplayed = 0
    current = 0
    
    
    #wait for board to be set up correctly
    current_state = capture_board_state(camera)
    print("Set up board")
 
    while not board_setup_complete(current_state):
        current_state = capture_board_state(camera)

    board = chess.Board()

    print("Ready to play!")
    #Send START command
    Start(spi)

    
    #Entering main loop
    while True:
        #Photograph board and decode state
        current_state = capture_board_state(camera)
        move = decode_move(board, current_state)
        if move != None:
            current_state = capture_board_state(camera)
            current_move = decode_move(board, current_state)
            if (current_move != None) and (current_move == move):
                board.push(move)
                result = engine.play(board, chess.engine.Limit(time=0.5))
                board.push(result.move)
                print(board)
                
                Move(spi, result.move)
                current = current + 2
                time.sleep(2)
                print("Waiting for done signal")
                while(DoneYet(spi) == 0):
                    time.sleep(2)
                    current_state = capture_board_state(camera)
                print("Please implement blue's move on the board")

                while(not np.array_equal(current_color_mask(board), current_state)):
                    current_state = capture_board_state(camera)
                print("Orange's move")
        
        else:
            #time.sleep(1)
            #check to see if user wants to have different state displayed on monitor
            displayed = CheckNew(spi)
            while(displayed > current):
                print(displayed)
                displayed = CheckNew(spi)
            if(displayed != prevdisplayed):
                #send new board state
                print("Current board state")
                print(current)
                print("displayed board state")
                print(displayed)
                print("sendboard")
                sendboard = previous_board(board, current, displayed)
                while(sendboard == None):
                    sendboard = previous_board(board, current, displayed)
                print(sendboard)
                BoardDisplay(spi, sendboard)
                prevdisplayed = displayed
           
                
                



if __name__ == '__main__':
	main()
