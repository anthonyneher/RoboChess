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
    
    #used to first capture does not update color spectrum
    update = np.empty((1952,2592,3), dtype=np.uint8)
    camera.capture(update, format='rgb')
    time.sleep(0.1)
    
    spi = init_spi()
    
    displayed = 0
    prevdisplayed = 0
    current = 0
    
    
    #wait for board to be set up correctly
    current_state = capture_board_state(camera)
    print("Set up board")
    print(current_state)
    
    while not board_setup_complete(current_state):
        current_state = capture_board_state(camera)
        print(current_state)

    board = chess.Board()

    print("Ready to play!")
    #Send START command
    Start(spi)

    
    #Entering main loop
    while True:
        time.sleep(1)
        #Photograph board and decode state
        current_state = capture_board_state(camera)
        """
        print("Waiting for valid move")
        print(board)
        print(current_state)
        """
        move = decode_move(board, current_state)
        if move != None:
            current_state = capture_board_state(camera)
            current_move = decode_move(board, current_state)
            if (current_move != None) and (current_move == move):
                board.push(move)
                result = engine.play(board, chess.engine.Limit(time=0.5))
                if board.piece_at(result.move.to_square):
                    Take(spi, result.move, int(board.piece_at(result.move.from_square).piece_type), int(board.piece_at(result.move.to_square).piece_type))
                else:
                    Move(spi, result.move, int(board.piece_at(result.move.from_square).piece_type))
                
                board.push(result.move)
                print(board)

                current = current + 2
                time.sleep(1)
                print("Waiting for done signal")
                DoneYet(spi)
                while(DoneYet(spi) == 0):
                    time.sleep(2)
                    #current_state = capture_board_state(camera)
                print("Done")

                while(not np.array_equal(current_color_mask(board), current_state)):
                    current_state = capture_board_state(camera)
                print("Orange's move")

        else:
            time.sleep(2)
            #check to see if user wants to have different state displayed on monitor
#            displayed= CheckNew(spi)
            displayed, reset_game, difficulty = CheckNew(spi)

            print("current " + str(current))
            print("displayed " + str(displayed))
            print("difficulty " + str(difficulty))
            print("reset game " + str(reset_game))
            
            while(displayed > current):
                time.sleep(3)
                print(displayed)
                displayed, reset_game, difficulty = CheckNew(spi)
 #               displayed = CheckNew(spi)
            if(displayed != prevdisplayed):
                #send new board state
                print("Current board state")
                print(current)
                print("displayed board state")
                print(displayed)
                print("sendboard")
                #sendboard = previous_board(board, current, current)
                sendboard = previous_board(board, current, displayed)
                while(sendboard == None):
                    sendboard = previous_board(board, current, displayed)
                print("board about to be updated")
                print(sendboard)
                #input("Send Board")
                BoardDisplay(spi, sendboard)
                prevdisplayed = displayed
           
                
                



if __name__ == '__main__':
	main()
