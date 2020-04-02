import chess
import chess.engine
#from chess import uci
import numpy as np
from copy import copy


BLACK = 2
WHITE = 1

def main():
    board = chess.Board()
    engine = chess.engine.SimpleEngine.popen_uci("/Users/anthonyneher/senior/stockfish-11-mac/src/stockfish")

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

    old = create_colormap()
    new = create_colormap()
    new[6][4] = 0
    new[5][4] = 1
    #new[3][4] = 1
    print("Old:")
    print(old)
    print("New:")
    print(new)
    ret = novel_state(old, new)
    print("Novel state returned: ", ret,"\n")

    board = chess.Board()
    print(board)
    print(new)
    #wait until new board state determined
    if(novel_state(old, new) == 1 and decode_move(board, new) != None):
        print("Check passed")
    else: print("Check failed")


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
    for y in range(8):
        for x in range(8):
            if previous[y][x] == 0 and current[y][x] != 0:
                prev_blank += 1
            if previous[y][x] != 0 and current[y][x] == 0:
                prev_colored += 1
    if prev_blank == 1 and prev_colored == 1:
        return 1
    elif prev_colored > 1:
        return -1
    elif prev_blank > 1:
        return -2


if __name__ == '__main__':
    main()


