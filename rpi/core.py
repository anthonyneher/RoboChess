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
    difficulty = 1
    
    
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
        time.sleep(0.1)
        #Photograph board and decode state
        current_state = capture_board_state(camera)
        """
        print("Waiting for valid move")
        print(board)
        print(current_state)
        """
        print("ENTRY CHECK")
        move = decode_move(board, current_state)
        if move != None:
            current_state = capture_board_state(camera)
            current_move = decode_move(board, current_state)
            if (current_move != None) and (current_move == move):
                board.push(move)
                if(current == displayed):
                    #update players resposne to screen
                    BoardDisplay(spi, board)
                    
                #calculate compuers response
                
                if difficulty == 1:
                    result = engine.play(board, chess.engine.Limit(time=0.001, depth = 1))
                elif difficulty == 2:
                    result = engine.play(board, chess.engine.Limit(time=0.05, depth = 5))                
                else:
                    result = engine.play(board, chess.engine.Limit(time=0.1, depth = 20))
                
                #result.move = chess.Move.from_uci("e8c8")
                #if False:
                if board.piece_at(result.move.to_square):
                    Take(spi, result.move, int(board.piece_at(result.move.from_square).piece_type), int(board.piece_at(result.move.to_square).piece_type))
                else:
                    if(board.piece_at(result.move.from_square).piece_type == chess.KING and (result.move.uci() == "e8g8" or result.move.uci() == "e8c8")):
                        print("Yeah")
                        if(result.move.uci() == "e8g8"):
                            print("Castle")
                            time.sleep(0.3)       
                            Castle(spi, KINGSIDE)
                            time.sleep(2)
                            while(DoneYet(spi) != 1):
                                time.sleep(1)
                        if(result.move.uci() == "e8c8"):
                            print("Queenside castle")
                            time.sleep(0.3)
                            Castle(spi, QUEENSIDE)
                            time.sleep(2)
                            while(DoneYet(spi) != 1):
                                time.sleep(1)
                    else:
                        print("else")
                        Move(spi, result.move, int(board.piece_at(result.move.from_square).piece_type))
                
                board.push(result.move)
                print(board)
                print(move)
                time.sleep(1)
                print("Waiting for done signal")
                while(DoneYet(spi) != 1):
                    time.sleep(2)
                    #current_state = capture_board_state(camera)
                print("Done")
                
                #update screen with response move
                if(current == displayed):
                    BoardDisplay(spi, board)
                current = current + 2
                
                while(not np.array_equal(current_color_mask(board), current_state)):
                    current_state = capture_board_state(camera)
                print("Orange's move")

        else:
            time.sleep(0.3)
            #check to see if user wants to have different state displayed on monitor
 
            if(CheckReset(spi) == 1):
                time.sleep(0.5)
                if not CheckReset(spi):
                    break
                print("RESETTING BOARD")
                time.sleep(0.25)
                board.reset()
                BoardDisplay(spi, board)#send reset board
                #reset global variables
                displayed = 0
                current = 0
                current_state = capture_board_state(camera)
                while not board_setup_complete(current_state):
                    current_state = capture_board_state(camera)
                    print("Waiting for board to be reset")
                    time.sleep(2)
            else:
                time.sleep(0.15)
                difficulty = CheckDif(spi)
                time.sleep(0.15)
                
                displayed= CheckNew(spi)
    #            displayed, reset_game, difficulty = CheckNew(spi)
                
                while(displayed > current):
                    time.sleep(3)
                    print(displayed)
     #               displayed, reset_game, difficulty = CheckNew(spi)
                    displayed = CheckNew(spi)
     
                #if(displayed != prevdisplayed):
                    #send new board state
                print("\n")
                print("Current board state")
                print(current)
                print("displayed board state")
                print(displayed)

                sendboard = previous_board(board, current, displayed)
                """
                while(sendboard == None):
                    sendboard = previous_board(board, current, displayed)
                """
                print("board about to be updated")
                time.sleep(0.15)
                #input("Send Board")
                BoardDisplay(spi, sendboard)
                #prevdisplayed = displayed


if __name__ == '__main__':
	main()
