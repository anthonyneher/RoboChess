import chess
import chess.engine
#from chess import uci
import numpy as np
from copy import copy
from comms import *


BLACK = 2
WHITE = 1

moves_played = 0;

def main():
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci("/usr/games/stockfish")

    #create test matrix
    positions = create_colormap()
    positions[6][4] = 0
    positions[4][4] = 1
    print(board)
    print("\n")
    move = decode_move(board, positions)
    board.push(move)
    print(board)

    result = engine.play(board, chess.engine.Limit(time=0.5))
    board.push(result.move)
    print("\n")
    print(board)
    engine.quit()
    print("\nTesting board novelty functions")

    print("first")
    print(previous_board(board, 2, 1))
    print("initial")
    print(previous_board(board, 2, 0))
    print("original")
    print(board)



#used to simplify testing
def create_colormap():
    positions = np.zeros((8,8), dtype=int)
    for y in range(8):
        for x in range(8):
            if y < 2:
                positions[7-y][x] = WHITE
            elif y > 5:
                positions[7-y][x] = BLACK
            else:
                positions[7-y][x] = 0
    return positions

#creates 0, 1, 2 color mask based on python-chess board state
def current_color_mask(board):
    positions = np.zeros((8,8),dtype=int)
    for y in range(8):
        for x in range(8):
            piece = board.piece_at(x+(y*8))
            if piece is not None:
                if piece.color is chess.WHITE:
                    positions[7-y][x] = WHITE
                else:
                    positions[7-y][x] = BLACK
            else:
                positions[7-y][x] = 0
    return positions


"""
    Iterates through all possible moves that can be made based on current python-chess board state and checks to see
    if the color mask of the board with each move applied matches the color mask detected by the camera. 
input:
    current_board - the current python-chess board state
    color_mask - numpy array of digits representing color mask of piece positions
output:
    move which resulted in new board state
"""
def decode_move(current_board, color_mask):
    for move in current_board.legal_moves:
        board = copy(current_board)
        board.push(move)
        if np.array_equal(current_color_mask(board), color_mask):
            return move
    return None


def previous_board(board, current, display):
    prev = copy(board)
    dif = (current-display)
    if (dif < 0 or dif > current):
        return None
    for x in range(dif):
        prev.pop()
    return prev

"""
    Determines whether the board state is new and valid by cross referencing previous state
input:
    previous valid color map
    current color map - can be invalid
output:
    1 - current board state is novel and valid
    -1 - too many blank squares on current board (ideally this is because arm is on board and move is being made)
    -2 - number of squares that were blank and now have a new piece is greater than 1
"""
def novel_state(previous, current):
    prev_blank = 0
    prev_colored = 0
    taken = 0
    for y in range(8):
        for x in range(8):
            pcolored = False
            pblank = False
            if previous[y][x] == 0 and current[y][x] != 0:
                prev_blank += 1
                pblank = True
            if previous[y][x] != 0 and current[y][x] == 0:
                prev_colored += 1
                pcolored = True
            if (previous[y][x] == 1 and current[y][x] == 2) or (previous[y][x] == 2 and current[y][x] == 1):
                if pcolored == True and pblank == False:
                    taken = taken + 1
    if (prev_blank == 1 and prev_colored == 1) or taken == 1:
        return 1
    elif prev_colored > 1:
        return -1
    elif prev_blank > 1:
        return -2
    elif taken > 1:
        "taken too big"
    else:
        return "no change"


if __name__ == '__main__':
    main()


